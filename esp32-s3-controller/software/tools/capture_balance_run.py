"""Capture a bounded, explicitly requested hardware balance run."""
import datetime
import pathlib
import re
import subprocess
import sys
import time

import serial


def run(output, mode='balance', duration=20):
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = 'COM30', 115200, .015
    port.dtr = port.rts = False
    with open(output, 'w', encoding='utf-8', buffering=1) as log:
        def read_for(seconds):
            chunks = []
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                data = port.read(16384)
                if data:
                    chunks.append(data)
            text = b''.join(chunks).decode('utf-8', 'replace')
            log.write(f'\n[host {time.monotonic():.3f}]\n{text}')
            return text

        try:
            port.open()
            port.reset_input_buffer()
            # Preserve the boot positioning PID history for the balance handoff.
            port.write((b'stop\n' if mode == 'spring' else b'') + b'imu off\nstatus\n')
            pre = read_for(.7)
            zero = all(re.search(rf'ID={i} target=0\.000 V ',pre) for i in range(1,7))
            holding = all(re.search(prefix + r'[^\r\n]*output=spring spring=(?:holding|position-holding)(?:\s|$)',pre)
                          for prefix in (r'LEG23 length=',r'LEG length='))
            if not ('CAN state=running' in pre and (mode == 'spring' or 'IMU ready ' in pre) and all(
                re.search(rf'ID={i} target=[-0-9.]+ V .*feedback=online', pre)
                for i in range(1, 7)
            ) and (zero or holding)):
                log.write('\nPRECHECK_REJECTED\n')
                return
            start = time.monotonic()
            log.write(f'\nSTART_COMMAND host={start:.3f}\n')
            port.write(b'spring\n' if mode == 'spring' else b'balance on\n')
            confirmed = False
            pending = ''
            while time.monotonic() - start < duration:
                if pathlib.Path(output + '.stop').exists():
                    log.write('\nEARLY_STOP requested\n')
                    break
                port.write(b'status\n')
                pending = (pending + read_for(.15))[-16000:]
                started = ('SPRING active:' in pending and 'LEG23 active:' in pending) if mode == 'spring' else 'BALANCE active: original LQR' in pending
                if not confirmed and started:
                    confirmed = True
                    log.write(f'\nCONTROL_STARTED_CONFIRMED elapsed={time.monotonic()-start:.3f}\n')
                if ('BALANCE rejected:' in pending or 'BALANCE waiting:' in pending or 'SPRING rejected:' in pending
                        or 'LEG23 rejected:' in pending or 'CAN bus-off' in pending
                        or (mode == 'balance' and 'IMU failed ' in pending) or 'Rebooting' in pending):
                    log.write('\nEARLY_STOP fault\n')
                    break
                if not confirmed and time.monotonic() - start > 2:
                    log.write('\nEARLY_STOP no start confirmation\n')
                    break
        finally:
            if port.is_open:
                port.write(b'stop\n')
                read_for(.3)
                port.write(b'status\n')
                post = read_for(.7)
                zero = all(re.search(rf'ID={i} target=0\.000 V ', post) for i in range(1, 7))
                log.write(f'\nFINISHED zero_targets_confirmed={zero}\n')
                port.close()


if __name__ == '__main__':
    if len(sys.argv) == 5 and sys.argv[1] == '--child':
        run(sys.argv[2], sys.argv[3], int(sys.argv[4]))
    else:
        mode = sys.argv[1] if len(sys.argv) > 1 else 'balance'
        duration = int(sys.argv[2]) if len(sys.argv) > 2 else (10 if mode == 'spring' else 20)
        if mode not in ('spring', 'balance') or not 1 <= duration <= 30:
            raise SystemExit('Expected: balance|spring [seconds, 1..30]')
        output = pathlib.Path(__file__).parent / (
            f'{mode}_diagnostic_{duration}s_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.log'
        )
        process = subprocess.Popen(
            [sys.executable, __file__, '--child', str(output), mode, str(duration)],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
        print(f'PID={process.pid} LOG={output}', flush=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            text = output.read_text(encoding='utf-8') if output.exists() else ''
            if 'CONTROL_STARTED_CONFIRMED' in text:
                print('CONTROL_STARTED_CONFIRMED', flush=True)
                break
            if process.poll() is not None:
                print(text[-4000:], flush=True)
                break
            time.sleep(.03)
