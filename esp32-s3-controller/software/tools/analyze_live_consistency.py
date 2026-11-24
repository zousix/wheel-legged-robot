"""Check recorded odometry and IMU bias before the operator unplugged USB.

Sparse console samples cannot establish high-frequency timing, ground slip,
true upright angle, or the physical cause of a collision.
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np

from analyze_wheel_feedback_delay import estimate, fit


def analyze(path):
    text = path.read_text(encoding='utf-8')
    text = text.split('"event": "balance_command"', 1)[1]
    text = text.split('"event": "serial_link_lost"', 1)[0]
    samples = {int(t): list(map(float, e.split(','))) for t, e in re.findall(
        r'LQR_SAMPLE control_us=(\d+) L_m=[-\d.]+ error=([-\d.,]+) T=', text)}
    rows = sorted((t, e) for t, e in samples.items() if len(e) == 6 and np.isfinite(e).all())
    times = (np.array([t for t, _ in rows])-rows[0][0])/1e6
    states = np.array([e for _, e in rows])
    dt = np.diff(times)
    dx = np.diff(states[:, 2])
    integral = .5*(states[1:, 3]+states[:-1, 3])*dt
    windows = []
    for lo, hi in [(5, 20), (28, 42), (48, 73), (85, 110), (125, 205), (220, 270)]:
        mask = (times >= lo) & (times <= hi)
        t, x, v = times[mask].tolist(), states[mask, 2].tolist(), states[mask, 3].tolist()
        frequency = max((fit(t, x, i*.005)[1], i*.005) for i in range(20, 201))[1]
        windows.append(dict(seconds=[lo, hi], **estimate(t, x, v, frequency)))
    bias = np.array([list(map(float, b.split(','))) for b in re.findall(
        r'IMU corrected_gyro\[dps\]=[-\d.,]+ bias\[dps\]=([-\d.,]+)', text)])
    gyro = np.array([list(map(float, g.split(','))) for g in re.findall(
        r'IMU corrected_gyro\[dps\]=([-\d.,]+) bias', text)])
    return dict(
        source=path.name, samples=len(rows), duration_s=float(times[-1]),
        sample_gap_s=dict(median=float(np.median(dt)), maximum=float(max(dt))),
        position_change_m=float(sum(dx)), integrated_reported_speed_m=float(sum(integral)),
        integral_disagreement_m=float(sum(dx-integral)),
        interval_displacement_speed_integral_correlation=float(np.corrcoef(dx, integral)[0, 1]),
        imu_bias_samples=len(bias),
        imu_bias_first_last_dps=[bias[0].tolist(), bias[-1].tolist()],
        imu_bias_range_dps=(bias.max(axis=0)-bias.min(axis=0)).tolist(),
        imu_bias_print_resolution_dps=.001,
        fraction_console_gyro_all_axes_within_3_dps=float(np.mean(np.max(np.abs(gyro), axis=1) <= 3)),
        harmonic_windows=windows,
        limits=[
            'Position and speed share encoders; consistency does not verify ground displacement or slip.',
            'Console rate cannot resolve fast vibration; integral mismatch includes undersampling.',
            'Harmonic phase is an effective phase estimate, not measured full actuator latency.',
            'Sparse gyro samples cannot prove a continuous five-second quiet interval.',
            'Bias is printed to 0.001 dps; smaller changes are unobservable.',
            'Unchanged bias does not prove incorrect bias; true upright reference is unmeasured.',
        ])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    result = analyze(args.log)
    destination = args.log.with_suffix('.consistency.json')
    destination.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
