"""Read-only, approximately 20 Hz status capture; never commands motor output."""
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path
from check_can import open_port, read_for

rows = []
logs = []
duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10
stem = 'project_response_' + datetime.now().strftime('%Y%m%d_%H%M%S')
with open_port('COM30') as connection:
    connection.timeout = 0.005
    connection.reset_input_buffer()
    start = time.monotonic()
    next_progress = 20
    while time.monotonic() - start < duration:
        connection.write(b'status\n')
        log = read_for(connection, 0.045)
        logs.append(log)
        poses = re.findall(r'LEG length=([0-9.]+) mm angle=([0-9.]+) deg[^\r\n]*spring=(\S+)', log)
        if not poses:
            continue
        length, angle, mode = poses[-1]
        row = dict(t=round(time.monotonic()-start, 3), length=float(length), angle=float(angle), mode=mode)
        for motor_id in (2, 3):
            motor = re.search(rf'ID={motor_id} target=([-0-9.]+) V angle=([-0-9.]+) rad speed=([-0-9.]+) rpm feedback=(\w+)', log)
            if motor:
                row.update({f'v{motor_id}': float(motor[1]), f'q{motor_id}': float(motor[2]),
                            f'rpm{motor_id}': float(motor[3]), f'online{motor_id}': motor[4]})
        rows.append(row)
        if time.monotonic() - start >= next_progress:
            print('Captured', round(time.monotonic()-start), 's;', len(rows), 'samples;',
                  'latest L=', length, 'angle=', angle, 'mode=', mode, flush=True)
            next_progress += 20
root = Path(__file__).parent
(root / 'project_response_latest.log').write_text(''.join(logs), encoding='utf-8')
(root / 'project_response_latest.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
(root / (stem + '.log')).write_text(''.join(logs), encoding='utf-8')
(root / (stem + '.json')).write_text(json.dumps(rows, indent=2), encoding='utf-8')
print('Saved:', stem)
print('Samples:', len(rows))
if rows:
    for key in ('length', 'angle', 'v2', 'v3', 'rpm2', 'rpm3'):
        values = [row[key] for row in rows if key in row]
        if values:
            print(key, 'min=', min(values), 'max=', max(values), 'last=', values[-1])
    print('Modes:', sorted({row['mode'] for row in rows}))
    print('Last samples:', json.dumps(rows[-5:]))
statuses = re.findall(r'CAN state=[^\r\n]+', ''.join(logs))
if statuses:
    print('First CAN:', statuses[0])
    print('Last CAN:', statuses[-1])
