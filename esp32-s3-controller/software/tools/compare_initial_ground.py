"""Offline comparison of equal-gain startup windows; never opens serial ports."""
import json
import math
from pathlib import Path
import re
import statistics as st

from analyze_lqr_samples import load_coefficients


def compare(path, coefficients):
    text = path.read_text(encoding='utf-8')
    before, body = text.split('EVENT {"event": "balance_command"', 1)
    t0 = int(re.search(r'BALANCE_TERMS t_ms=(\d+).*control_us=0', body)[1])*1000
    rows, terms = {}, {}
    cutoff = None
    for line in body.splitlines():
        if line.startswith('BALANCE_TERMS'):
            f = dict(re.findall(r'(\w+)=([^\s]+)', line))
            terms[int(f['control_us'])] = f
        if not line.startswith('LQR_SAMPLE'):
            continue
        f = dict(re.findall(r'(\w+)=([^\s]+)', line))
        ts = int(f['control_us'])
        t = (ts-t0)/1e6
        if t >= 50:
            cutoff = body.index(line)
            break
        e = list(map(float, f['error'].split(',')))
        length = float(f['L_m'])
        ratio, pos, rate, speed = [float(f[k]) for k in (
            'gain_ratio', 'wheel_position_ratio', 'wheel_body_rate_ratio', 'wheel_speed_ratio')]
        k = [(((a*length+b)*length+c)*length+d)*ratio for a,b,c,d in (coefficients[i] for i in range(12))]
        parts = [k[2*i]*e[i]*(pos if i == 2 else speed if i == 3 else 1) for i in range(6)]
        parts[5] += (rate-1)*(k[10]-k[2])*e[5]
        assert abs(sum(parts)-float(f['T'])) < 3e-5, 'LQR reconstruction mismatch'
        r = dict(t=t, gains=(ratio,pos,rate,speed), **dict(zip(
            ('theta','dtheta','x','speed','pitch','rate'),e)))
        r['v1'], r['v6'] = map(float, f['V_ID1_ID6'].split(','))
        r['legs'] = list(map(float, f['legs_mm'].split(',')))
        r['angles'] = list(map(float, f['angles_deg'].split(',')))
        r['parts'] = dict(theta=parts[0],position=parts[2],speed=parts[3],pitch=parts[4],
                          joint_rate=k[2]*(e[1]+e[5]),body_rate=rate*(k[10]-k[2])*e[5])
        z = terms.get(ts)
        if z:
            r.update(gap_us=float(z['gap_us']),imu_age_us=float(z['imu_age_us']),
                     jointV=list(map(float,z['V1_6'].split(',')))[1:5],roll_N=float(z['roll_N']))
        rows[ts] = r
    rows = list(rows.values())
    windows = {}
    for a,b in ((0,5),(5,20),(20,50),(0,50)):
        rr = [r for r in rows if a <= r['t'] < b]
        assert len(rr) > 100, 'Insufficient samples'
        sd = lambda key: st.pstdev(r[key] for r in rr)
        w = dict(samples=len(rr),gains=sorted(set(r['gains'] for r in rr)),
                 wheel_voltage_std_V=math.sqrt((sd('v1')**2+sd('v6')**2)/2),
                 wheel_cap_fraction=sum(max(abs(r['v1']),abs(r['v6']))>=3.99 for r in rr)/len(rr),
                 speed_std_mps=sd('speed'),pitch_std_deg=math.degrees(sd('pitch')),
                 body_rate_std_dps=math.degrees(sd('rate')),theta_std_deg=math.degrees(sd('theta')),
                 position_span_cm=100*(max(r['x'] for r in rr)-min(r['x'] for r in rr)),
                 legs_mean_mm=[st.mean(r['legs'][i] for r in rr) for i in range(2)],
                 leg_std_mm=[st.pstdev(r['legs'][i] for r in rr) for i in range(2)],
                 component_std_Nm={k:st.pstdev(r['parts'][k] for r in rr) for k in rr[0]['parts']})
        assert w['gains'] == [(0.3,0.8,0.75,1.0)], 'Early comparison gains differ'
        valid = [r for r in rr if 'jointV' in r]
        assert valid
        w['joint_cap_fraction'] = [sum(abs(r['jointV'][i])>=6.89 for r in valid)/len(valid) for i in range(4)]
        for key in ('gap_us','imu_age_us'):
            v = sorted(r[key] for r in valid)
            w[key] = dict(samples=len(v),median=st.median(v),p99=v[int(.99*(len(v)-1))],maximum=max(v))
        windows[f'{a}..{b}s'] = w
    return dict(log=path.name,t0_control_us=t0,windows=windows,
                early_CAN_errors=dict(bus_off=body[:cutoff].count('CAN bus-off'),
                                      transmit_failed=body[:cutoff].count('CAN transmit failed')),
                last_PID={side:re.findall(r'^LEG_PID side='+side+r'.*$',before,re.M)[-1] for side in ('23','54')},
                last_prebalance_body=re.findall(r'^IMU body:.*$',before,re.M)[-1],
                last_prebalance_pose=re.findall(r'^STATE legs:.*$',before,re.M)[-1],
                first_handoff=re.findall(r'^BALANCE_LENGTH.*$',body,re.M)[0])


if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    coeff = load_coefficients(root.parent/'main/lqr_k.c')
    results = {stem:compare(root/(stem+'.log'),coeff) for stem in (
        'live_speed_20261022_182611','live_speed_20261022_191504')}
    output = root/'initial_ground_comparison_20261022_191504.json'
    output.write_text(json.dumps(results,indent=2),encoding='utf-8')
    for name,result in results.items():
        print(name, json.dumps(result,ensure_ascii=False),flush=True)
    print('PASS: unique timestamps, sufficient samples, LQR sum and unchanged early gains checked.')
