"""Summarize disturbances; peak-to-return is NOT a measured release-to-settle time."""
import json
import re
from pathlib import Path

root = Path(__file__).parent
rows = json.loads((root / 'project_response_latest.json').read_text())
target = 128.2
print('N=', len(rows), 'span=', rows[-1]['t']-rows[0]['t'])
for key in ('length', 'angle', 'v2', 'v3', 'rpm2', 'rpm3'):
    values = [row[key] for row in rows if key in row]
    print(key, min(values), max(values))
print('Events: start_s, peak_s, peak_mm, first_return_s, peak_to_return_s, next_1s_min/max_mm')
i = 0
while i < len(rows):
    if abs(rows[i]['length']-target) < 5:
        i += 1
        continue
    start = i
    direction = 1 if rows[i]['length'] > target else -1
    j = i + 1
    while j < len(rows) and direction*(rows[j]['length']-target) > 2:
        j += 1
    peak = max(range(start, j), key=lambda k: direction*(rows[k]['length']-target))
    if j == len(rows):
        print('Incomplete event', rows[start]['t'], rows[peak]['t'], rows[peak]['length'])
        break
    following = [r['length'] for r in rows[j:] if r['t'] <= rows[j]['t']+1]
    print(round(rows[start]['t'],2), round(rows[peak]['t'],2), rows[peak]['length'],
          round(rows[j]['t'],2), round(rows[j]['t']-rows[peak]['t'],2),
          min(following), max(following))
    for k in range(peak, len(rows)):
        window = [r for r in rows[k:] if r['t'] <= rows[k]['t']+0.5]
        if window[-1]['t']-window[0]['t'] >= 0.45 and all(abs(r['length']-target)<=2 for r in window):
            print('  Within +/-2 mm for 0.5 s starts at', rows[k]['t'],
                  'peak-to-settle=', round(rows[k]['t']-rows[peak]['t'],3))
            break
    print('  First return pose:', rows[j]['length'], rows[j]['angle'])
    i = j + 1
tail = [row for row in rows if row['t'] >= rows[-1]['t']-5]
print('Final 5 s:', {key: (min(r[key] for r in tail), max(r[key] for r in tail))
                     for key in ('length', 'angle')})
print('Modes:', sorted({r['mode'] for r in rows}))
for motor in (2, 3):
    print('Motor', motor, 'saturation_samples=',sum(abs(r.get(f'v{motor}',0))>=3.99 for r in rows),
          'offline_samples=',sum(r.get(f'online{motor}')=='offline' for r in rows))
log = (root / 'project_response_latest.log').read_text()
statuses = re.findall(r'CAN state=[^\r\n]+', log)
print('CAN first:', statuses[0])
print('CAN last:', statuses[-1])
