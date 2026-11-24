"""Offline integration checks with simulated serial and time. No motor I/O."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import capture_live_speed_tuning as cap


def simulate(duration,ready_after=0,fault_stage='balance'):
    instances=[]
    clock=SimpleNamespace(value=0.)
    def tick():
        clock.value+=.005
        return clock.value
    def telemetry(angle=90):
        text=('IMU ready id=0x6b\nIMU body: left_roll=0 forward_pitch=0\n'
              f'STATE legs: ID2/3 L=128.2 mm angle={angle} deg rate=0 rad/s; '
              f'ID5/4 L=128.2 mm angle={angle} deg rate=0 rad/s\n')
        text+=''.join(f'ID={i} target=0.000 V angle=0 rad speed=0 rpm feedback=online\n'for i in range(1,7))
        text+=''.join(f'LEG_PID side={side} t_ms=1 age_ms=1 L_mm=128.2 dl_mps=0\n'for side in ('23','54'))
        return text
    fault='W (4651564) foc_can: CAN bus-off; recovery started\n'
    class FakeSerial:
        def __init__(self):
            self.buffer=b'';self.commands=[];self.closed=False
            self.spring_time=None;self.balance_time=None;self.samples=0;self.final_read=False
            instances.append(self)
        def open(self):assert self.dtr is False and self.rts is False
        def flush(self):pass
        def write(self,data):
            cmd=data.decode().strip();self.commands.append(cmd)
            if cmd=='wheel zero':reply='WHEELS zero complete; RAM only\n'
            elif cmd.startswith('balance rate '):reply=f'BALANCE rate={float(cmd.split()[-1]):.3f}; RAM only\n'
            elif cmd.startswith('balance speed '):reply=f'BALANCE speed={float(cmd.split()[-1]):.3f}; RAM only\n'
            elif cmd=='spring':self.spring_time=clock.value;reply='SPRING active\n'
            elif cmd=='balance on':self.balance_time=clock.value;reply='BALANCE active phase=standing-control\n'
            elif cmd=='balance sample':
                self.samples+=1
                reply=('LQR_SAMPLE control_us=4000 L_m=0.1282 error=0,0,0,0,0,0 T=0 Tp=0 torque_ID1_ID6=0,0 V_ID1_ID6=0,0 gain_ratio=0.300 wheel_position_ratio=0.800 wheel_body_rate_ratio=0.660 wheel_speed_ratio=1.000\n'if self.samples==1 else fault)
            elif cmd=='stop\nstatus':reply=fault+'BALANCE off\n'+telemetry()
            elif self.spring_time is not None:
                elapsed=clock.value-self.spring_time
                reply=fault if fault_stage=='pid'and elapsed>2 else telemetry(86 if elapsed<ready_after else 90)
            else:reply=telemetry()
            self.buffer=reply.encode();return len(data)
        def read(self,size):
            data=self.buffer[:size];self.buffer=self.buffer[size:]
            if data and self.commands[-1]=='stop\nstatus':self.final_read=True
            return data
        def close(self):self.closed=True
    stdout=io.StringIO()
    with tempfile.TemporaryDirectory()as folder:
        with patch.object(cap.serial,'Serial',FakeSerial),patch.object(cap,'time',SimpleNamespace(monotonic=tick)),patch.object(cap,'__file__',str(Path(folder)/'capture.py')),contextlib.redirect_stdout(stdout):
            cap.run(continuous=True,body_rate=.66,pid_seconds=duration)
        files=[p for p in Path(folder).glob('live_speed_*.json')if not p.name.endswith('.command.json')]
        assert len(files)==1
        report=json.loads(files[0].read_text(encoding='utf-8'))
    serial=instances[0]
    assert serial.closed and serial.final_read and serial.commands[-1]=='stop\nstatus'
    assert 'STOP_UNCONFIRMED'not in stdout.getvalue()
    if ready_after>=999 or fault_stage=='pid':
        assert serial.balance_time is None
        assert report['pid_duration_seconds']is None
    else:
        assert report['pid_duration_seconds']>=duration
        assert serial.balance_time-serial.spring_time>=duration
        if ready_after:
            assert serial.balance_time-serial.spring_time>=ready_after+.6
    return report


if __name__=='__main__':
    cases=[simulate(10),simulate(30),simulate(30,ready_after=32),
           simulate(30,ready_after=999),simulate(30,fault_stage='pid')]
    assert cases[3]['reason']=='PID not ready within 40 s'
    assert cases[4]['reason'].startswith('CAN communication fault:')
    for invalid in (9,float('nan'),float('inf')):
        with patch.object(cap.serial,'Serial',side_effect=AssertionError('must not open serial')):
            try:cap.run(continuous=True,body_rate=.66,pid_seconds=invalid)
            except ValueError:pass
            else:raise AssertionError('invalid PID duration accepted')
    path=Path(__file__).with_name('capture_pid_duration_verification.json')
    path.write_text(json.dumps(dict(mode='offline simulation, no hardware I/O',cases=cases),indent=2),encoding='utf-8')
    print('PASS: default10s, diagnostic30s, delayed readiness, failed readiness, CAN failure during PID, final stop/status/close, and invalid input.')
