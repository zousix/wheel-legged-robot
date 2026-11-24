"""Validate and export stopped-only 4 ms balance trace snapshots; no serial I/O."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics

from analyze_lqr_samples import load_coefficients

COLUMNS = ('control_us', 'gap_us', 'imu_age_us', 'wheel_left_age_us',
           'wheel_right_age_us', 'theta_rad', 'dtheta_rad_s', 'position_error_m',
           'speed_mps', 'backward_pitch_rad', 'backward_pitch_rate_rad_s',
           'T_Nm', 'Tp_Nm', 'mean_length_m', 'length_PID_N', 'roll_N',
           'hip_left_Nm', 'hip_right_Nm', 'V1', 'V2', 'V3', 'V4', 'V5', 'V6')


def analyze(path, k_path):
    text = path.read_text(encoding='utf-8')
    header = re.search(r'BALANCE_TRACE_BEGIN count=(\d+) capacity=(\d+) '
                       r'gain_ratio=([\d.]+) position_ratio=([\d.]+) body_rate_ratio=([\d.]+)(?: speed_ratio=([\d.]+))?', text)
    if not header:
        raise ValueError('No complete trace header')
    count, capacity = map(int, header.groups()[:2])
    gain_ratio, position_ratio, rate_ratio = map(float, header.groups()[2:5])
    speed_ratio=float(header[6]) if header[6] is not None else 1.0
    tail = text[header.end():]
    end = re.search(r'BALANCE_TRACE_END count=(\d+)', tail)
    if not end or int(end[1]) != count or 'BALANCE_TRACE_ABORT' in tail[:end.start()]:
        raise ValueError('Incomplete or aborted trace')
    lines = re.findall(r'^BT,([^\r\n]*)', tail[:end.start()], re.M)
    if not 0 < count <= capacity or len(lines) != count:
        raise ValueError('Trace count mismatch or empty capture')
    samples = [list(map(float, line.split(','))) for line in lines]
    if any(len(row) != len(COLUMNS) or not all(map(math.isfinite, row)) for row in samples):
        raise ValueError('Malformed/nonfinite trace row')
    rows = [dict(zip(COLUMNS, values)) for values in samples]
    gaps = [b['control_us']-a['control_us'] for a, b in zip(rows, rows[1:])]
    if not gaps or min(gaps) <= 0 or any(gap != row['gap_us'] for gap, row in zip(gaps, rows[1:])):
        raise ValueError('Trace chronology/gap mismatch')
    coefficients = load_coefficients(k_path)
    for row in rows:
        length = row['mean_length_m']
        gains = [(((a*length+b)*length+c)*length+d)*gain_ratio
                 for a, b, c, d in (coefficients[i] for i in range(12))]
        errors = [row[name] for name in COLUMNS[5:11]]
        wheel = sum(gains[2*i]*errors[i]*(position_ratio if i == 2 else speed_ratio if i == 3 else 1) for i in range(6))
        wheel += (rate_ratio-1)*(gains[10]-gains[2])*errors[5]
        hip = sum(gains[2*i+1]*errors[i] for i in range(6))
        if max(abs(wheel-row['T_Nm']), abs(hip-row['Tp_Nm'])) > 3e-5:
            raise ValueError('Trace/gain-source mismatch at '+str(row['control_us']))
    output = path.with_suffix('.trace.csv')
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(samples=count, span_s=(rows[-1]['control_us']-rows[0]['control_us'])/1e6,
                   gap_us=dict(min=min(gaps), max=max(gaps), mean=statistics.mean(gaps)),
                   gain_ratio=gain_ratio, position_ratio=position_ratio, body_rate_ratio=rate_ratio, speed_ratio=speed_ratio,
                   imu_age_us_max=max(row['imu_age_us'] for row in rows),
                   wheel_age_us_max=max(max(row['wheel_left_age_us'], row['wheel_right_age_us']) for row in rows))
    path.with_suffix('.trace.summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('PASS: complete chronological trace and synchronized LQR reconstruction', summary)
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--k-source', type=Path, default=Path(__file__).resolve().parents[1]/'main/lqr_k.c')
    args = parser.parse_args()
    analyze(args.trace, args.k_source)
