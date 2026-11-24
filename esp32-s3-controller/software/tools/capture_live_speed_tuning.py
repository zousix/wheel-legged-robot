"""One PID-to-balance run; compare wheel-speed feedback with sparse serial capture.

Existing startup checks only; no runtime leg-angle stop. Always stop on exit.
The comparison is sequential, not a controlled identification of the plant.
"""
import json
import argparse
import math
from pathlib import Path
import re
import statistics
import time
from datetime import datetime
import serial
from capture_guarded_balance import Telemetry


def check_can_line(line):
    if 'foc_can: CAN bus-off' in line or 'foc_can: CAN transmit failed:' in line:
        raise RuntimeError('CAN communication fault: '+line.strip())


def metrics(rows):
    if len(rows)<30:
        raise RuntimeError('not enough synchronized samples')
    def sd(key):
        return statistics.pstdev(row[key] for row in rows)
    return dict(samples=len(rows), voltage_std=math.sqrt((sd('v1')**2+sd('v6')**2)/2),
                voltage_rms=math.sqrt(statistics.mean((r['v1']**2+r['v6']**2)/2 for r in rows)),
                wheel_cap_fraction=sum(abs(r['v1'])>=3.99 or abs(r['v6'])>=3.99 for r in rows)/len(rows),
                speed_mean=statistics.mean(r['speed'] for r in rows),
                speed_rms=math.sqrt(statistics.mean(r['speed']**2 for r in rows)),
                speed_std=sd('speed'), position_span=max(r['x'] for r in rows)-min(r['x'] for r in rows),
                position_mean=statistics.mean(r['x'] for r in rows), pitch_std_deg=sd('pitch')*180/math.pi,
                quiet_fraction=sum(abs(r['speed'])<.01 for r in rows)/len(rows))


def run(continuous=False,body_rate=.8,pid_seconds=10):
    if not math.isfinite(body_rate) or not .5<=body_rate<=1:
        raise ValueError('body rate must be finite and within existing firmware range 0.5..1.0')
    if not math.isfinite(pid_seconds) or pid_seconds<10:
        raise ValueError('PID recording duration must be finite and at least 10 s')
    stem='live_speed_'+datetime.now().strftime('%Y%m%d_%H%M%S')
    root=Path(__file__).parent
    log=(root/(stem+'.log')).open('w',encoding='utf-8')
    conn=serial.Serial();conn.port='COM30';conn.baudrate=115200;conn.timeout=.005
    conn.write_timeout=1;conn.dtr=conn.rts=False
    telemetry=Telemetry();tail='';samples={};reason='';reports={};chosen=1.0;opened=False
    monitor_can=False
    pid_actual=None

    def read(command,seconds=.10):
        nonlocal tail
        if command:
            conn.write((command+'\n').encode());conn.flush()
        deadline=time.monotonic()+seconds;out=''
        while time.monotonic()<deadline:
            text=conn.read(16384).decode(errors='replace');out+=text;log.write(text);log.flush()
            telemetry.feed(text,time.monotonic())
            lines=(tail+text.replace('\r','')).split('\n');tail=lines.pop()
            for line in lines:
                if monitor_can:check_can_line(line)
                if line.startswith('LQR_SAMPLE'):
                    fields=dict(re.findall(r'(\w+)=([^\s]+)',line))
                    err=list(map(float,fields['error'].split(',')))
                    volts=list(map(float,fields['V_ID1_ID6'].split(',')))
                    samples[int(fields['control_us'])]=dict(time=time.monotonic(),
                        gain=float(fields['wheel_speed_ratio']),x=err[2],speed=err[3],pitch=err[4],
                        theta=err[0],body_rate=err[5],v1=volts[0],v6=volts[1])
        return out.replace('\r','')

    def event(name,**kw):
        line='EVENT '+json.dumps(dict(event=name,host_s=time.monotonic(),**kw))
        log.write('\n'+line+'\n');log.flush();print(line,flush=True)

    def stage(gain,duration=15):
        reply=read(f'balance speed {gain:.3f}',.2)
        if f'BALANCE speed={gain:.3f};' not in reply:
            raise RuntimeError('speed command not acknowledged')
        samples.clear();start=time.monotonic();last=start
        event('speed_stage',gain=gain)
        while time.monotonic()-start<duration:
            text=read('balance sample',.05)
            if 'BALANCE_SAMPLE unavailable' in text:
                raise RuntimeError('balance output unavailable')
            if time.monotonic()-last>=5:
                rows=[r for r in samples.values() if r['gain']==gain and r['time']-start>2]
                if len(rows)>=30: print('WINDOW',gain,json.dumps(metrics(rows)),flush=True)
                last=time.monotonic()
        rows=[r for r in samples.values() if r['gain']==gain and r['time']-start>2]
        result=metrics(rows);event('stage_result',gain=gain,metrics=result);return result

    try:
        conn.open();opened=True
        read('stop\nimu off',.25)
        pre=read('status',.2)
        if 'IMU failed' in pre:read('imu restart',6)
        for _ in range(12):
            if 'zero complete' in read('wheel zero',.2):break
        else:raise RuntimeError('wheel reference unavailable')
        reply=read(f'balance rate {body_rate:.3f}',.2)
        if f'BALANCE rate={body_rate:.3f};' not in reply:
            raise RuntimeError('body-rate command not acknowledged')
        read('balance speed 1.0',.2)
        pre=read('status\nstate',.15);now=time.monotonic()
        if 'IMU ready' not in pre or telemetry.fault(now):
            raise RuntimeError('preflight not ready: '+str(telemetry.fault(now)))
        pose=telemetry.get('pose',now)
        if any(not 70<pose[i]<110 for i in (1,4)):
            raise RuntimeError('initial legs not pointing down')
        monitor_can=True
        event('spring_command',pid_minimum_seconds=pid_seconds);read('spring',.2)
        start=time.monotonic();stable=None;shown=0
        pid_deadline=max(20,pid_seconds+10)
        while time.monotonic()-start<pid_deadline:
            read('status\nstate',.10);now=time.monotonic()
            stable=(now if stable is None else stable) if telemetry.ready(now) else None
            if now-shown>2:print('PID POSE',telemetry.get('pose',now),flush=True);shown=now
            if now-start>=pid_seconds and stable is not None and now-stable>=.6:break
        else:raise RuntimeError(f'PID not ready within {pid_deadline:g} s')
        pid_actual=time.monotonic()-start
        event('balance_command',pid_duration_seconds=pid_actual)
        reply=read('balance on',.2)
        if 'BALANCE active' not in reply:raise RuntimeError('balance entry rejected')
        if continuous:
            command_path=root/(stem+'.command.json')
            command_path.write_text(json.dumps(dict(sequence=0,command=None)),encoding='utf-8')
            event('continuous_ready',command_file=str(command_path),speed=1.0,body_rate=body_rate)
            sequence=0;window=time.monotonic();samples.clear()
            while True:
                text=read('balance sample',.05)
                if 'BALANCE_SAMPLE unavailable' in text:
                    raise RuntimeError('balance output unavailable')
                try:
                    command=json.loads(command_path.read_text(encoding='utf-8'))
                except (OSError,ValueError):
                    command={}
                if command.get('sequence',0)>sequence:
                    sequence=command['sequence'];name=command.get('command')
                    if name=='stop':reason='operator stop';return
                    if name in ('speed','rate'):
                        value=float(command['value'])
                        if not math.isfinite(value) or value<=0:raise ValueError('invalid live gain')
                        reply=read(f'balance {name} {value:.3f}',.2)
                        event('live_command',command=name,value=value,ack=reply.strip())
                        if f'BALANCE {name}={value:.3f};' not in reply:
                            raise RuntimeError('live command not acknowledged')
                        if name=='speed':chosen=value
                        else:body_rate=value
                        window=time.monotonic();samples.clear()
                if time.monotonic()-window>=5:
                    rows=list(samples.values());report=metrics(rows)
                    report['theta_std_deg']=statistics.pstdev(r['theta'] for r in rows)*180/math.pi
                    report['body_rate_std_dps']=statistics.pstdev(r['body_rate'] for r in rows)*180/math.pi
                    event('continuous_window',speed=chosen,body_rate=body_rate,metrics=report)
                    samples.clear();window=time.monotonic()
        for gain in (1.0,.95,.9):
            reports[gain]=stage(gain)
        baseline=reports[1.0]
        # Retain a trial only with less speed oscillation and no clear worsening of
        # wheel chatter, pitch oscillation or position excursion in this same run.
        candidates=[g for g,m in reports.items() if g!=1.0 and
                    m['speed_std']<baseline['speed_std']*.9 and
                    m['voltage_std']<=baseline['voltage_std']*1.1 and
                    m['pitch_std_deg']<=baseline['pitch_std_deg']*1.1 and
                    m['position_span']<=baseline['position_span']*1.1]
        if candidates:chosen=min(candidates,key=lambda g:reports[g]['speed_std'])
        event('selected_speed',gain=chosen)
        reports['confirmation']=stage(chosen)
        reason='four live stages complete'
    except KeyboardInterrupt:
        reason='operator interrupt'
    except Exception as error:
        reason=str(error)
    finally:
        monitor_can=False
        if opened:
            try:
                final=read('stop\nstatus',.4)
                print('\n'.join(x for x in final.splitlines() if x.startswith(('ID=','BALANCE off'))),flush=True)
            except Exception as error:print('STOP_UNCONFIRMED',repr(error),flush=True)
            conn.close()
        log.close()
        summary=dict(reason=reason,selected_speed=chosen,body_rate=body_rate,reports=reports,
                     pid_minimum_seconds=pid_seconds,pid_duration_seconds=pid_actual,log=stem+'.log')
        (root/(stem+'.json')).write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print('RESULT',json.dumps(summary),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--continuous',action='store_true',help='Keep balancing until an explicit stop or communication/control fault; no timed stop')
    parser.add_argument('--body-rate',type=float,default=.8,help='Initial wheel body-rate contribution (RAM only)')
    parser.add_argument('--pid-seconds',type=float,default=10,help='Minimum preparation PID recording time; default10 s, existing readiness conditions still required')
    args=parser.parse_args();run(continuous=args.continuous,body_rate=args.body_rate,pid_seconds=args.pid_seconds)
