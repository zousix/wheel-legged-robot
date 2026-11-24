"""Offline phase estimate from synchronized angle-derived position and wheel speed.

Fits one slow oscillation plus a linear drift. This is not an identification of
the full actuator delay; sparse samples and other oscillations bias the result.
"""
import argparse
import json
import math
from pathlib import Path
import re


def solve(matrix, vector):
    a = [list(row) + [v] for row, v in zip(matrix, vector)]
    for i in range(len(vector)):
        pivot = max(range(i, len(vector)), key=lambda j: abs(a[j][i]))
        a[i], a[pivot] = a[pivot], a[i]
        divisor = a[i][i]
        if abs(divisor) < 1e-12:
            raise ValueError('Singular harmonic fit')
        a[i] = [v / divisor for v in a[i]]
        for j in range(len(vector)):
            if j != i:
                factor = a[j][i]
                a[j] = [x - factor*y for x, y in zip(a[j], a[i])]
    return [row[-1] for row in a]


def fit(times, values, frequency):
    center = sum(times)/len(times)
    mean = sum(values)/len(values)
    rows = [[1, t-center, math.cos(2*math.pi*frequency*t),
             math.sin(2*math.pi*frequency*t)] for t in times]
    matrix = [[sum(r[i]*r[j] for r in rows) for j in range(4)] for i in range(4)]
    vector = [sum(r[i]*v for r, v in zip(rows, values)) for i in range(4)]
    coefficients = solve(matrix, vector)
    residual = sum((v-sum(c*x for c, x in zip(coefficients, r)))**2
                   for r, v in zip(rows, values))
    total = sum((v-mean)**2 for v in values)
    return coefficients, 1-residual/total


def estimate(times, position, speed, frequency):
    x, position_r2 = fit(times, position, frequency)
    v, speed_r2 = fit(times, speed, frequency)
    w = 2*math.pi*frequency
    response = complex(v[2], -v[3])/complex(w*x[3], w*x[2])
    return dict(fit_frequency_hz=frequency, position_fit_R2=position_r2,
                speed_fit_R2=speed_r2,
                phase_delay_estimate_ms=-1000*math.atan2(response.imag, response.real)/w,
                speed_to_position_derivative_gain=abs(response),
                weak_fit=min(position_r2, speed_r2) < .5)


def analyze(path):
    text = path.read_text(encoding='utf-8').split('"event": "balance_command"')[1]
    text = text.split('"event": "stop_command"')[0]
    samples = {int(t): list(map(float, e.split(','))) for t, e in re.findall(
        r'LQR_SAMPLE control_us=(\d+) L_m=[-\d.]+ error=([-\d.,]+)', text)}
    start = min(samples)
    rows = [(t, e) for t, e in sorted(samples.items()) if t-start > 1000000]
    times = [(t-start)/1e6 for t, _ in rows]
    position, speed = ([e[i] for _, e in rows] for i in (2, 3))
    frequency = max((fit(times, position, i*.005)[1], i*.005)
                    for i in range(20, 201))[1]
    return dict(samples=len(rows), **estimate(times, position, speed, frequency))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='*', type=Path)
    args = parser.parse_args()
    times = [i*.017+(i % 3)*.001 for i in range(1000)]
    w, delay = 2*math.pi*.45, .02
    result = estimate(times, [.1*math.cos(w*t) for t in times],
                      [-.1*w*math.sin(w*(t-delay)) for t in times], .45)
    assert abs(result['phase_delay_estimate_ms']-20) < 1e-6
    assert abs(result['speed_to_position_derivative_gain']-1) < 1e-6
    print('PASS: known 20 ms delayed velocity recovered from irregular samples')
    for path in args.logs:
        result = analyze(path)
        print(path.name, json.dumps(result))
        path.with_suffix('.delay_fit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
