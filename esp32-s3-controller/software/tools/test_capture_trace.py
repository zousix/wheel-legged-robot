"""Simulated serial/clock test: dump trace only after stop and zero confirmation."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile

import capture_guarded_balance as capture


class Connection:
    def __init__(self, zero_confirmed, trace_rows=2):
        self.clock = 0.
        self.pending = b''
        self.stopped = self.spring = self.balancing = False
        self.zero_confirmed = zero_confirmed
        self.commands = []
        self.trace_rows = trace_rows
        self.rate = .8
        self.closed = False

    def now(self):
        self.clock += .002
        return self.clock

    def open(self):
        pass

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        pass

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, size):
        data, self.pending = self.pending[:size], self.pending[size:]
        return data

    def write(self, command):
        self.commands.append(command)
        if command == b'spring\n':
            self.spring = True
        if command == b'balance on\n':
            self.balancing = True
        if command == b'stop\n':
            self.stopped = True
        if command == b'balance trace\n':
            assert self.stopped and self.zero_confirmed
            text = f'BALANCE_TRACE_BEGIN count={self.trace_rows}\n'
            text += ''.join(f'BT,{i}\n' for i in range(self.trace_rows))
            text += f'BALANCE_TRACE_END count={self.trace_rows}\n'
            self.pending += text.encode()
            return
        if command.startswith(b'balance rate '):
            self.rate = float(command.split()[2])
        text = ('PID lo kp=15 ki=0.05 kd=0 imax=30 max=1\n'
                'PID li kp=20 ki=0.1 kd=30 imax=30 max=10\n'
                'IMU ready\nBALANCE off ground_est=0\n'
                'WHEELS valid left=ID1/+1 right=ID6/-1\n'
                'IMU body: left_roll=0 forward_pitch=0\n'
                'STATE legs: ID2/3 L=128.2 mm angle=90 deg rate=0 rad/s; '
                'ID5/4 L=128.2 mm angle=90 deg rate=0\n')
        text += f'BALANCE rate={self.rate:.3f}; RAM only\n'
        for motor in range(1, 7):
            voltage = '.5' if self.stopped and not self.zero_confirmed and motor == 1 else '0.000'
            text += f'ID={motor} target={voltage} V feedback=online\n'
        if self.spring and not self.stopped:
            text += 'LEG_PID side=23 age_ms=0 dl_mps=0\nLEG_PID side=54 age_ms=0 dl_mps=0\n'
        if self.balancing and not self.stopped:
            text = text.replace('BALANCE off ground_est=0', 'BALANCE active phase=standing-control ground_est=1')
            text += (f'LQR_SAMPLE control_us={int(self.clock*1e6)} error=0,0,0,0.1,0.01,0 '
                     f'V_ID1_ID6={self.rate},-{self.rate} wheel_body_rate_ratio={self.rate:.3f}\n')
        self.pending += text.encode()


capture.check_recorded_log()
capture.check_preflight_policy()
original_clock = capture.time.monotonic
original_serial = capture.serial.Serial
original_file = capture.__file__
original_stdin = capture.sys.stdin
original_thread = capture.threading.Thread
original_queue = capture.queue.Queue
try:
    capture.check_recorded_log = capture.check_preflight_policy = lambda: None
    for zero_confirmed, trace_rows in ((True, 2), (False, 2), (True, 0)):
        with tempfile.TemporaryDirectory() as directory:
            connection = Connection(zero_confirmed, trace_rows)
            capture.serial.Serial = lambda: connection
            capture.time.monotonic = connection.now
            capture.__file__ = str(Path(directory)/'capture.py')
            with redirect_stdout(io.StringIO()):
                capture.run(capture_trace=True)
            result = json.loads(next(Path(directory).glob('*.json')).read_text())
            assert result['balance_sent'] and result['reason'] == '30 s balance window complete'
            events = {e['event']: e['host_s'] for e in result['events']}
            assert 5 <= events['balance_command']-events['spring_command'] < 5.3
            assert 30 <= events['stop_command']-events['balance_command'] < 30.2
            assert result['trace']['complete'] == (zero_confirmed and trace_rows > 0)
            assert (b'balance trace\n' in connection.commands) == zero_confirmed
            assert connection.closed
    class ImmediateThread:
        def __init__(self, target, daemon):
            self.target = target
        def start(self):
            self.target()
    class ContinuousConnection(Connection):
        failed_once = interrupted = False
        def write(self, command):
            if self.balancing and self.clock > 15 and not self.failed_once:
                self.failed_once = True
                live_commands.put('0.8')  # Old decision pending while USB is unavailable.
                raise capture.serial.SerialException('simulated temporary USB disconnect')
            if self.balancing and self.clock > 45 and not self.interrupted:
                self.interrupted = True
                raise KeyboardInterrupt
            super().write(command)
    with tempfile.TemporaryDirectory() as directory:
        connection = ContinuousConnection(True)
        capture.serial.Serial = lambda: connection
        capture.time.monotonic = connection.now
        capture.__file__ = str(Path(directory)/'capture.py')
        capture.sys.stdin = io.StringIO('nan\n0.6\n0.75\n')
        capture.threading.Thread = ImmediateThread
        live_commands = original_queue()
        capture.queue.Queue = lambda: live_commands
        output = io.StringIO()
        with redirect_stdout(output):
            capture.run(capture_trace=True, live_rate=True)
        result = json.loads(next(Path(directory).glob('*.json')).read_text())
        changes = [e for e in result['events'] if e['event'] == 'rate_command']
        assert len(changes) == 1 and changes[0]['rate'] == .75
        assert connection.commands.count(b'spring\n') == connection.commands.count(b'balance on\n') == 1
        assert b'balance rate 0.750\n' in connection.commands
        assert output.getvalue().count('LIVE_REJECTED') == 2
        windows = [json.loads(line.split(' ', 1)[1]) for line in output.getvalue().splitlines() if line.startswith('LIVE_WINDOW ')]
        assert windows and all(w['samples'] > 0 and w['rate'] == .75 for w in windows)
        events = {e['event']: e['host_s'] for e in result['events']}
        assert events['stop_command']-events['balance_command'] > 30
        assert events['serial_link_restored'] > events['serial_link_lost']
        assert any(e['event'] == 'stale_commands_discarded' and e['count'] == 1 for e in result['events'])
        assert result['reason'] == 'operator interrupt'
        assert connection.commands.count(b'stop\n') == 1
    class FailedFinalStatus(Connection):
        def write(self, command):
            if self.stopped and command == b'status\n':
                raise RuntimeError('simulated USB disconnect')
            super().write(command)
    with tempfile.TemporaryDirectory() as directory:
        connection = FailedFinalStatus(True)
        capture.serial.Serial = lambda: connection
        capture.time.monotonic = connection.now
        capture.__file__ = str(Path(directory)/'capture.py')
        with redirect_stdout(io.StringIO()):
            capture.run(capture_trace=True)
        result = json.loads(next(Path(directory).glob('*.json')).read_text())
        assert 'simulated USB disconnect' in result['capture_error'] and connection.closed
finally:
    capture.time.monotonic = original_clock
    capture.serial.Serial = original_serial
    capture.__file__ = original_file
    capture.sys.stdin = original_stdin
    capture.threading.Thread = original_thread
    capture.queue.Queue = original_queue
print('PASS: continuous live tuning exceeds 30 s; USB reconnect preserves control until operator stop; batch deadline retained; no hardware opened')
