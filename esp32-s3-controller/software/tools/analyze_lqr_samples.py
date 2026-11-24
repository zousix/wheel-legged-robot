"""Reconstruct each LQR contribution from synchronized firmware output; no serial I/O."""
import argparse
import csv
from pathlib import Path
import re


def load_coefficients(k_path):
    source = k_path.read_text(encoding='utf-8')
    coefficients = {}
    for index, expression in re.findall(r'K\[(\d+)\] = (.*?);', source):
        values = re.findall(r'([-+]?\d+\.\d+e[-+]\d+)f', expression, re.I)
        if len(values) == 4:
            coefficients[int(index)] = list(map(float, values))
    if set(coefficients) != set(range(12)):
        raise ValueError('Expected the current 12 cubic gain polynomials')
    return coefficients


def analyze(log_path, k_path):
    coefficients = load_coefficients(k_path)
    samples = {}
    pattern = r'LQR_SAMPLE control_us=(\d+) L_m=([-\d.]+) error=([-\d.,]+) T=([-\d.]+) Tp=([-\d.]+) torque_ID1_ID6=([-\d.,]+) V_ID1_ID6=([-\d.,]+)(?: gain_ratio=([-\d.]+))?(?: wheel_position_ratio=([-\d.]+))?(?: wheel_body_rate_ratio=([-\d.]+))?'
    names = ('theta', 'dtheta', 'position', 'speed', 'body_pitch', 'body_rate')
    capture=log_path.read_text(encoding='utf-8')
    for match in re.finditer(pattern, capture):
        timestamp, length, errors, total, hip, torque, volts, ratio, position_ratio, body_rate_ratio = match.groups()
        suffix=capture[match.end():].split('\n',1)[0]
        speed_match=re.search(r'wheel_speed_ratio=([-\d.]+)',suffix)
        speed_ratio=float(speed_match[1]) if speed_match else 1.0
        body_rate_ratio = float(body_rate_ratio) if body_rate_ratio is not None else 1.0
        position_ratio = float(position_ratio) if position_ratio is not None else 1.0
        ratio = float(ratio) if ratio is not None else 1.0
        length = float(length)
        errors, torque, volts = (list(map(float, vector.split(','))) for vector in (errors, torque, volts))
        if len(errors) != 6 or len(torque) != 2 or len(volts) != 2:
            continue
        gain = [(((a*length+b)*length+c)*length+d)*ratio for a, b, c, d in (coefficients[i] for i in range(12))]
        wheel_parts = [gain[2*i]*errors[i]*(position_ratio if i == 2 else speed_ratio if i == 3 else 1.0) for i in range(6)]
        wheel_parts[5] += (body_rate_ratio-1.0)*(gain[10]-gain[2])*errors[5]
        hip_parts = [gain[2*i+1]*errors[i] for i in range(6)]
        if max(abs(sum(wheel_parts)-float(total)), abs(sum(hip_parts)-float(hip))) > 3e-5:
            raise ValueError(f'Gain/source or capture mismatch at control_us={timestamp}')
        row = dict(control_us=int(timestamp), length_m=length, gain_ratio=ratio, wheel_position_ratio=position_ratio, wheel_body_rate_ratio=body_rate_ratio, T_Nm=float(total), Tp_Nm=float(hip),
                   torque_ID1_Nm=torque[0], torque_ID6_Nm=torque[1], V_ID1=volts[0], V_ID6=volts[1])
        row['wheel_speed_ratio']=speed_ratio
        row.update((f'T_{name}_Nm', value) for name, value in zip(names, wheel_parts))
        row.update((f'Tp_{name}_Nm', value) for name, value in zip(names, hip_parts))
        # dtheta includes -dphi: separate joint-rate feedback from the shared body rate.
        row['T_joint_rate_Nm'] = gain[2]*(errors[1]+errors[5])
        row['T_body_rate_combined_Nm'] = body_rate_ratio*(gain[10]-gain[2])*errors[5]
        samples[int(timestamp)] = row
    if not samples:
        raise ValueError('No complete synchronized LQR samples; old STATE logs cannot substitute')
    output = log_path.with_suffix('.lqr.csv')
    rows = [samples[timestamp] for timestamp in sorted(samples)]
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'Validated {len(rows)} synchronized samples; contributions: {output}')
    for prefix in ('T', 'Tp'):
        ranking = sorted(names, key=lambda name: sum(abs(row[f'{prefix}_{name}_Nm']) for row in rows), reverse=True)
        print(prefix+' mean absolute contributions [Nm]: '+', '.join(
            f'{name}={sum(abs(row[f"{prefix}_{name}_Nm"]) for row in rows)/len(rows):.6f}' for name in ranking))
    print('Wheel rate split mean absolute [Nm]: '+', '.join(
        f'{name}={sum(abs(row[name]) for row in rows)/len(rows):.6f}'
        for name in ('T_joint_rate_Nm', 'T_body_rate_combined_Nm')))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--k-source', type=Path, default=Path(__file__).resolve().parents[1]/'main/lqr_k.c')
    args = parser.parse_args()
    analyze(args.log, args.k_source)
