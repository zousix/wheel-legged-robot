"""Explicitly start spring control and save six seconds of initial status."""
import re
import time
from pathlib import Path
from check_can import open_port, read_for

with open_port('COM30') as connection:
    connection.write(b'spring\n')
    deadline = time.monotonic() + 6
    next_status = 0
    data = bytearray()
    pending = ''
    stopped = False
    while time.monotonic() < deadline:
        if time.monotonic() >= next_status:
            connection.write(b'status\n')
            next_status = time.monotonic() + 0.25
        chunk = connection.read(4096)
        data.extend(chunk)
        pending += chunk.decode(errors='replace')
        lines = pending.split('\n')
        pending = lines.pop()
        for line in lines:
            pose = re.search(r'LEG length=([0-9.]+) mm angle=([0-9.]+) deg', line)
            if pose and (not 85 <= float(pose[1]) <= 156 or abs(float(pose[2])-90) > 45):
                connection.write(b'stop\n')
                stopped = True
        if stopped:
            break
    connection.write(b'status\n')
    data.extend(read_for(connection, 1).encode())
    log = data.decode(errors='replace')
    Path(__file__).with_name('project_control_first_spring.log').write_text(log, encoding='utf-8')
    lines = [line.strip() for line in log.splitlines() if line.strip()]
    print('\n'.join(lines[:15]))
    print('Latest status:')
    print('\n'.join(lines[-18:]))
    print('HOST_STOP=', stopped)
