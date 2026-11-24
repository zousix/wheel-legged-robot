"""Ground control: PID at least 10 s, then 30 s test or continuous live tuning.

Run only after the operator has confirmed ground placement and fall restraint.
Runtime telemetry is recorded without pose/feedback early stops; preflight checks remain.
Telemetry fields arrive independently; incomplete USB lines never erase fresh data.
"""
import json
import argparse
import re
import math
import queue
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial


class Telemetry:
    def __init__(self):
        self.tail = ''
        self.fields = {}

    def feed(self, text, now):
        lines = (self.tail + text.replace('\r', '')).split('\n')
        self.tail = lines.pop()
        for line in lines:
            match = re.search(r'STATE legs: ID2/3 L=([\d.]+) mm angle=([-\d.]+) deg rate=([-\d.]+) rad/s; ID5/4 L=([\d.]+) mm angle=([-\d.]+) deg rate=([-\d.]+)', line)
            if match:
                self.fields['pose'] = (now, tuple(map(float, match.groups())))
            match = re.search(r'LEG_PID side=(23|54).*?age_ms=([\d.]+).*?dl_mps=([-\d.]+)', line)
            if match:
                self.fields['pid' + match[1]] = (now, tuple(map(float, match.groups()[1:])))
            match = re.search(r'IMU body: left_roll=([-\d.]+) forward_pitch=([-\d.]+)', line)
            if match:
                self.fields['body'] = (now, tuple(map(float, match.groups())))
            match = re.search(r'ID=([1-6]) target=([-\d.]+) V.*?feedback=(online|offline)', line)
            if match:
                self.fields['motor' + match[1]] = (now, (float(match[2]), match[3]))
            match = re.search(r'WHEELS left_x=.*?left_v=([-\d.]+) m/s right_v=([-\d.]+)', line)
            if match:
                self.fields['wheels'] = (now, tuple(map(float, match.groups())))
            if 'BALANCE active phase=' in line:
                self.fields['phase'] = (now, line)
            match = re.search(r'LQR_SAMPLE control_us=(\d+).*?error=([-\d.,]+).*?V_ID1_ID6=([-\d.,]+).*?wheel_body_rate_ratio=([\d.]+)', line)
            if match:
                errors = list(map(float, match[2].split(',')))
                volts = list(map(float, match[3].split(',')))
                if len(errors) == 6 and len(volts) == 2:
                    self.fields['lqr'] = (now, (int(match[1]), float(match[4]), *volts, errors[4], errors[3]))

    def get(self, key, now, age=.25):
        sample = self.fields.get(key)
        return sample[1] if sample and now - sample[0] <= age else None

    def ready(self, now):
        pose = self.get('pose', now)
        pid = [self.get('pid' + side, now) for side in ('23', '54')]
        return bool(pose and all(p is not None for p in pid)
                    and all(abs(pose[i] - 128.2) <= 10 for i in (0, 3))
                    and all(abs(pose[i] - 90) < 3 for i in (1, 4))
                    and all(abs(pose[i]) < .2 for i in (2, 5))
                    and all(p[0] < 30 and abs(p[1]) < .02 for p in pid))

    def fault(self, now, balancing=False):
        body, pose = self.get('body', now), self.get('pose', now)
        if body is None or pose is None:
            return 'body/leg telemetry timeout'
        if max(map(abs, body)) > 15:
            return 'body tilt >15 deg'
        if any(not (70 < pose[i] < 155 and 45 < pose[i+1] < 135) for i in (0, 3)):
            return 'leg pose protection'
        for i in range(1, 7):
            motor = self.get('motor' + str(i), now)
            if motor is None or motor[1] != 'online':
                return 'motor feedback unavailable'
        if balancing:
            wheels = self.get('wheels', now)
            if wheels is None:
                return 'wheel telemetry timeout'
            phase = self.get('phase', now)
            if phase and 'pose-pause' in phase:
                return 'firmware pose protection'
        return None


def check_recorded_log():
    path = Path(__file__).with_name('standing_balance_max5s_20261019_175420.log')
    text = path.read_text(encoding='utf-8').split('"event": "spring_command"', 1)[1].split('"event": "stop_command"', 1)[0]
    telemetry, stable, recognized = Telemetry(), None, False
    now = start = 0.0
    for line in text.splitlines(keepends=True):
        match = re.search(r'LEG_PID side=\d+ t_ms=(\d+)', line)
        if match:
            now = max(now, float(match[1]) / 1000)
            if not start:
                start = now
        # Exercise USB splitting as well as separately arriving status/state lines.
        for i in range(0, len(line), 13):
            telemetry.feed(line[i:i+13], now)
        stable = (now if stable is None else stable) if telemetry.ready(now) else None
        if stable is not None and now - start >= 5 and now - stable >= .6:
            recognized = True
    assert recognized, 'recorded qualifying pose was not recognized'
    assert not telemetry.ready(now + .3), 'stale cached pose was accepted'
    print('PASS: split/asynchronous recorded telemetry recognizes readiness; stale data rejected', flush=True)


def check_preflight_policy():
    t = Telemetry()
    now = 1.0
    t.fields = {'body': (now, (0., 0.)),
                'pose': (now, (128.2, 90., 0., 128.2, 90., 0.)),
                'wheels': (now, (2., -2.))}
    for i in range(1, 7):
        t.fields['motor'+str(i)] = (now, (0., 'online'))
    assert t.fault(now, True) is None, 'wheel speed must not stop this test'
    t.fields['body'] = (now, (16., 0.))
    assert t.fault(now, True) == 'body tilt >15 deg'
    t.fields['body'] = (now, (0., 0.))
    t.fields['pose'] = (now, (69., 90., 0., 128.2, 90., 0.))
    assert t.fault(now, True) == 'leg pose protection'
    t.fields['pose'] = (now, (128.2, 90., 0., 128.2, 90., 0.))
    t.fields['motor1'] = (now, (0., 'offline'))
    assert t.fault(now, True) == 'motor feedback unavailable'
    assert t.fault(now+.3, True) == 'body/leg telemetry timeout'
    print('PASS: preflight body/leg/fresh-feedback checks retained; no wheel-speed limit', flush=True)


def live_summary(samples):
    rows = list(samples.values())
    if not rows:
        return dict(samples=0)
    return dict(samples=len(rows), rate=rows[-1][1],
                wheel_std_V=[statistics.pstdev(row[i] for row in rows) for i in (2, 3)],
                wheel_cap_fraction=[sum(abs(row[i]) >= 3.99 for row in rows)/len(rows) for i in (2, 3)],
                pitch_std_deg=statistics.pstdev(math.degrees(row[4]) for row in rows),
                speed_rms_mps=math.sqrt(sum(row[5]**2 for row in rows)/len(rows)))


def run(capture_trace=False, live_rate=False, balance_seconds=30):
    if not math.isfinite(balance_seconds) or not 0 < balance_seconds <= 60:
        raise ValueError('balance_seconds must be finite and within (0, 60]')
    check_recorded_log()
    check_preflight_policy()
    telemetry, logs, events = Telemetry(), [], []
    stem = ('standing_balance_live_' if live_rate else 'standing_balance_timed30s_') + datetime.now().strftime('%Y%m%d_%H%M%S')
    root = Path(__file__).parent
    connection = serial.Serial()
    connection.port, connection.baudrate, connection.timeout = 'COM30', 115200, .002
    connection.write_timeout = 1
    connection.dtr = connection.rts = False
    connection.open()
    reason, balance_sent = '', False
    trace_result = None
    final, capture_error = '', None
    commands = queue.Queue()
    stop_requested = threading.Event()
    log_stream = (root/(stem+'.log')).open('w', encoding='utf-8')
    def record(text):
        if log_stream:
            log_stream.write(text)
            log_stream.flush()
        else:
            logs.append(text)
    if live_rate:
        def read_commands():
            for line in sys.stdin:
                if line.strip() == 'stop':
                    stop_requested.set()
                commands.put(line.strip())
        threading.Thread(target=read_commands, daemon=True).start()

    def event(name, **details):
        item = dict(event=name, host_s=time.monotonic(), **details)
        events.append(item)
        record('\nHOST_EVENT ' + json.dumps(item) + '\n')
        print('STAGE', name, 'host_s='+str(item['host_s']), flush=True)

    def read_for(seconds):
        end, raw = time.monotonic() + seconds, bytearray()
        try:
            while time.monotonic() < end:
                raw.extend(connection.read(max(1, connection.in_waiting)))
        finally:
            text = raw.decode(errors='replace')
            record(text)
            telemetry.feed(text, time.monotonic())
        return text

    def reconnect_live():
        connection.close()
        telemetry.fields.clear()
        telemetry.tail = ''
        event('serial_link_lost')
        print('LIVE_LINK: reconnecting; tuning paused, no stop/start/reset sent', flush=True)
        while not stop_requested.is_set():
            try:
                connection.open()
                connection.write(b'status\nstate\nbalance rate\n')
                text = read_for(.3)
                if 'BALANCE off phase=' in text:
                    raise RuntimeError('balance already off after reconnect; no automatic restart')
                rate = re.search(r'BALANCE rate=([\d.]+);', text)
                online = all(re.search(rf'ID={i} .*?feedback=online', text) for i in range(1, 7))
                if 'BALANCE active phase=' in text and 'IMU ready' in text and online and rate:
                    discarded = 0
                    while not commands.empty():
                        commands.get_nowait()
                        discarded += 1
                    if discarded:
                        event('stale_commands_discarded', count=discarded)
                    # stop_requested remains set even if its queued line was discarded.
                    event('serial_link_restored', rate=float(rate[1]))
                    return float(rate[1])
            except serial.SerialException:
                pass
            connection.close()
            time.sleep(.5)
        raise KeyboardInterrupt

    try:
        connection.reset_input_buffer()
        connection.write(b'pid\nstatus\nstate\n')
        pre = read_for(.2)
        assert 'ground_est=' in pre and 'IMU ready' in pre, 'firmware/IMU not ready'
        assert 'WHEELS valid left=ID1/+1 right=ID6/-1' in pre, 'wheel mapping mismatch'
        assert 'PID lo kp=15 ki=0.05 kd=0 imax=30 max=1' in pre, 'outer PID mismatch'
        assert 'PID li kp=20 ki=0.1 kd=30 imax=30 max=10' in pre, 'inner PID mismatch'
        assert telemetry.fault(time.monotonic()) is None, 'preflight telemetry/pose failed'
        assert all(telemetry.get('motor'+str(i), time.monotonic())[0] == 0 for i in range(1, 7)), 'motors not stopped'
        pose = telemetry.get('pose', time.monotonic())
        assert all(70 < pose[i] < 160 and 70 < pose[i+1] < 110 for i in (0, 3)), 'initial leg pose unsuitable'
        if live_rate:
            connection.write(b'balance rate\n')
            reply = read_for(.15)
            rate_match = re.search(r'BALANCE rate=([\d.]+);', reply)
            assert rate_match, 'firmware has no live rate command'
            current_rate = float(rate_match[1])
        event('spring_command')
        connection.write(b'spring\n')
        start, stable = time.monotonic(), None
        while time.monotonic() - start < 20:
            connection.write(b'status\nstate\n')
            read_for(.085)
            now = time.monotonic()
            stable = (now if stable is None else stable) if telemetry.ready(now) else None
            if now-start >= 10 and stable is not None and now-stable >= .6:
                event('balance_command')
                telemetry.fields.pop('phase', None)
                connection.write(b'balance on\n')
                balance_sent, deadline = True, math.inf if live_rate else time.monotonic()+balance_seconds
                window_start, live_samples = time.monotonic(), {}
                if live_rate:
                    print('LIVE_READY: continuous until stop; send rate (step <=0.05, range 0.5..1.0) or stop', flush=True)
                while time.monotonic() < deadline:
                    if live_rate and stop_requested.is_set():
                        reason = 'operator live stop'
                        break
                    if live_rate and not commands.empty():
                        command = commands.get_nowait()
                        if command == 'stop':
                            reason = 'operator live stop'
                            break
                        try:
                            requested = round(float(command), 3)
                            assert math.isfinite(requested) and .5 <= requested <= 1
                            assert abs(requested-current_rate) <= .050001
                        except (ValueError, AssertionError):
                            print('LIVE_REJECTED: rate must be finite, 0.5..1.0 and step <=0.05', flush=True)
                        else:
                            event('rate_command', rate=requested)
                            try:
                                connection.write(f'balance rate {requested:.3f}\n'.encode())
                                current_rate = requested
                            except serial.SerialException:
                                current_rate = reconnect_live()
                            window_start, live_samples = time.monotonic(), {}
                    try:
                        connection.write(b'status\nstate\n')
                        text = read_for(min(.03, max(0, deadline-time.monotonic())))
                    except serial.SerialException:
                        if not live_rate:
                            raise
                        current_rate = reconnect_live()
                        window_start, live_samples = time.monotonic(), {}
                        continue
                    if 'BALANCE waiting:' in text or 'BALANCE rejected:' in text:
                        reason = 'balance entry rejected'
                        break
                    if live_rate:
                        now = time.monotonic()
                        sample = telemetry.get('lqr', now)
                        if sample and abs(sample[1]-current_rate) < .0001 and now-window_start >= .5:
                            live_samples[sample[0]] = sample
                        if now-window_start >= 5:
                            report = dict(host_s=now, **live_summary(live_samples))
                            record('\nLIVE_WINDOW '+json.dumps(report)+'\n')
                            print('LIVE_WINDOW', json.dumps(report), flush=True)
                            window_start, live_samples = now, {}
                reason = reason or f'{balance_seconds:g} s balance window complete'
                break
        reason = reason or 'PID not ready within 20 s; balance not sent'
    except KeyboardInterrupt:
        reason = 'operator interrupt'
    except Exception as error:
        reason = 'preflight/error: ' + str(error)
    finally:
        event('stop_command')
        try:
            connection.write(b'stop\n')
            read_for(.2)
            connection.write(b'status\n')
            final = read_for(.25)
            if capture_trace and balance_sent:
                targets=re.findall(r'ID=([1-6]) target=([-\d.]+) V',final)
                if len(dict(targets))==6 and all(float(v)==0 for _,v in targets):
                    event('trace_dump')
                    connection.write(b'balance trace\n')
                    deadline=time.monotonic()+30
                    dump=''
                    while time.monotonic()<deadline:
                        dump+=read_for(.1)
                        if any(marker in dump for marker in ('BALANCE_TRACE_END','BALANCE_TRACE_ABORT','BALANCE_TRACE rejected')):
                            break
                    (root/(stem+'.trace.log')).write_text(dump,encoding='utf-8')
                    end=re.search(r'BALANCE_TRACE_END count=(\d+)',dump)
                    rows=re.findall(r'^BT,[^\r\n]*',dump,re.M)
                    trace_result=dict(complete=bool(end and len(rows)==int(end[1]) and len(rows)>0),rows=len(rows))
                    print('TRACE',trace_result,flush=True)
                else:
                    trace_result=dict(complete=False,error='zero targets not confirmed; dump skipped')
        except Exception as error:
            capture_error = repr(error)
            print('STOP_OR_DUMP_ERROR', capture_error, flush=True)
        finally:
            connection.close()
            if log_stream:
                log_stream.close()
            else:
                (root/(stem+'.log')).write_text(''.join(logs), encoding='utf-8')
            (root/(stem+'.json')).write_text(json.dumps(dict(reason=reason, balance_sent=balance_sent, events=events, trace=trace_result, capture_error=capture_error), indent=2), encoding='utf-8')
        print('RESULT', reason, 'BALANCE_SENT', balance_sent, 'STEM', stem)
        print('EVENTS', events)
        print('\n'.join(line for line in final.splitlines() if line.startswith(('ID=', 'BALANCE', 'LEG', 'IMU body:'))))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',action='store_true',help='After stop, dump 4 ms samples from trace-capable firmware')
    parser.add_argument('--live-rate',action='store_true',help='Continuous until stop; stdin rate changes, 5 s metrics, USB reconnect without stopping control')
    args=parser.parse_args()
    run(capture_trace=args.trace, live_rate=args.live_rate)
