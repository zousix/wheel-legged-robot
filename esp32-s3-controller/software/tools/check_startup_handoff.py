"""Bounded startup diagnosis on the saved firmware; logs open before actuation."""
import json
import math
from pathlib import Path
import re
import time

import serial
from analyze_lqr_samples import load_coefficients
from capture_guarded_balance import Telemetry

ROOT = Path(__file__).resolve().parent
COEFFICIENTS = load_coefficients(ROOT.parent / 'main/lqr_k.c')


def wheel_torque(length, errors):
    k = [(((a*length+b)*length+c)*length+d)*.3
         for a, b, c, d in (COEFFICIENTS[i] for i in range(12))]
    total = sum(k[2*i]*errors[i]*(.8 if i == 2 else 1) for i in range(6))
    return total - .2*(k[10]-k[2])*errors[5]


def predict(text):
    models = re.findall(r'STATE model: theta=([-\d.]+) rad dtheta=([-\d.]+) rad/s '
                        r'x=([-\d.]+) m dx=([-\d.]+) m/s phi=([-\d.]+) rad dphi=([-\d.]+)', text)
    poses = re.findall(r'STATE legs: ID2/3 L=([\d.]+) mm.*?ID5/4 L=([\d.]+) mm', text)
    if not models or not poses:
        return None
    errors = list(map(float, models[-1]))
    errors[2] = 0  # Firmware reanchors wheel position at the handoff.
    length = sum(map(float, poses[-1])) / 2000
    torque = wheel_torque(length, errors)
    return dict(length_m=length, errors=errors, T_Nm=torque,
                static_wheel_voltage_V=max(abs(torque/.0101), abs(torque/.0096)))


def self_check():
    log = (ROOT/'standing_balance_timed30s_20261020_175442.log').read_text(encoding='utf-8')
    sample = re.search(r'LQR_SAMPLE control_us=\d+ L_m=([\d.]+) error=([^ ]+) T=([-\d.]+)', log)
    assert sample
    actual = wheel_torque(float(sample[1]), list(map(float, sample[2].split(','))))
    assert abs(actual-float(sample[3])) < 3e-5
    assert predict('partial serial line') is None
    print('PASS: first saved runnable LQR sample reconstructed; partial state rejected', flush=True)


def run():
    self_check()
    stem = ROOT / time.strftime('startup_handoff_check_%Y%m%d_%H%M%S')
    connection = serial.Serial()
    connection.port, connection.baudrate = 'COM30', 115200
    connection.timeout, connection.write_timeout = .01, 1
    connection.dtr = connection.rts = False
    telemetry = Telemetry()
    events, predictions = [], []
    reason, balance_sent = '', False
    final = ''
    with stem.with_suffix('.log').open('w', encoding='utf-8') as log:
        def event(name):
            item = dict(event=name, host_s=time.monotonic())
            events.append(item)
            log.write('\nHOST_EVENT '+json.dumps(item)+'\n')
            log.flush()
            print('STAGE', name, flush=True)

        def read_for(seconds):
            deadline, chunks = time.monotonic()+seconds, []
            while time.monotonic() < deadline:
                chunks.append(connection.read(max(1, connection.in_waiting)).decode(errors='replace'))
            text = ''.join(chunks)
            log.write(text)
            log.flush()
            telemetry.feed(text, time.monotonic())
            return text

        try:
            connection.open()
            connection.reset_input_buffer()
            connection.write(b'stop\nstatus\nstate\n')
            pre = read_for(.35)
            now = time.monotonic()
            assert 'IMU ready' in pre and 'CAN state=running tx_err=0 rx_err=0' in pre
            assert telemetry.fault(now) is None, 'preflight pose/feedback unavailable'
            assert all(telemetry.get('motor'+str(i), now)[0] == 0 for i in range(1, 7))
            event('spring_command')
            connection.write(b'spring\n')
            start, stable = time.monotonic(), None
            while time.monotonic()-start < 8:
                connection.write(b'status\nstate\n')
                text = read_for(.08)
                now = time.monotonic()
                predicted = predict(text)
                if predicted:
                    predicted['host_s'] = now
                    predictions.append(predicted)
                # This diagnostic isolates a nearly upright, low-command entry.
                # It does not change firmware gains or the regular test policy.
                body = telemetry.get('body', now)
                good = (telemetry.ready(now) and telemetry.fault(now) is None
                        and body and abs(body[0]) <= 5 and abs(body[1]) <= 1
                        and predicted and abs(predicted['errors'][5]) < .2
                        and predicted['static_wheel_voltage_V'] <= .8)
                stable = (now if stable is None else stable) if good else None
                if now-start >= 5 and stable is not None and now-stable >= .6:
                    event('balance_command_100ms')
                    connection.write(b'balance on\n')
                    balance_sent = True
                    deadline = time.monotonic()+.1
                    while time.monotonic() < deadline:
                        connection.write(b'status\nstate\n')
                        read_for(min(.015, max(0, deadline-time.monotonic())))
                    reason = '100 ms handoff capture complete'
                    break
            reason = reason or 'PID-only: no nearly-upright low-command entry within 8 s'
        except Exception as error:
            reason = 'halted: '+repr(error)
        finally:
            if connection.is_open:
                try:
                    event('stop_command')
                    connection.write(b'stop\n')
                    read_for(.15)
                    connection.write(b'status\n')
                    final = read_for(.3)
                finally:
                    connection.close()
            stem.with_suffix('.json').write_text(json.dumps(dict(
                reason=reason, balance_sent=balance_sent, events=events,
                predictions=predictions), indent=2), encoding='utf-8')
    print('RESULT', reason, 'LOG', stem, flush=True)
    print('\n'.join(line for line in final.splitlines()
                    if line.startswith(('CAN state', 'ID=', 'LEG', 'IMU ready', 'IMU failed', 'IMU body'))))


if __name__ == '__main__':
    run()
