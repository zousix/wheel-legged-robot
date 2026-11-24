"""Reconstruct current VMC commands from rounded serial snapshots (approximate)."""
import json
import math as m
from pathlib import Path

rows = json.loads(Path(__file__).with_name('last_disturbance.json').read_text())
for r in rows:
    p1 = m.pi / 2 - (r['q3'] + 6.421)
    p4 = m.pi / 2 - (r['q2'] + 2.458)
    # Equal-radius circle intersection on the hanging-leg branch.
    ax, ay = -.03 + .05*m.cos(p1), .05*m.sin(p1)
    bx, by = .03 + .05*m.cos(p4), .05*m.sin(p4)
    dx, dy = bx-ax, by-ay
    d = m.hypot(dx, dy)
    h = m.sqrt(.105**2 - d*d/4)
    x, y = (ax+bx)/2 - h*dy/d, (ay+by)/2 + h*dx/d
    length, angle = m.hypot(x,y), m.atan2(y,x)
    ux, uy, vx, vy = x-ax, y-ay, x-bx, y-by
    det = ux*vy-uy*vx
    a = .05*(-ux*m.sin(p1)+uy*m.cos(p1))
    b = .05*(-vx*m.sin(p4)+vy*m.cos(p4))
    cx, cy = [a*vy/det,-b*uy/det], [-a*vx/det,b*ux/det]
    jl = [(x*cx[i]+y*cy[i])/length for i in range(2)]
    jt = [(x*cy[i]-y*cx[i])/length**2 for i in range(2)]
    vel = [-r['rpm3']*m.pi/30,-r['rpm2']*m.pi/30]
    dl, dt = sum(jl[i]*vel[i] for i in range(2)), sum(jt[i]*vel[i] for i in range(2))
    err = .1282-length
    force = (180+140*min(abs(err)/.020,1))*err-20*dl-1.30
    ae = m.pi/2-angle
    friction = m.copysign(.020,ae) if abs(r['rpm2'])<2 and abs(r['rpm3'])<2 and abs(ae)>m.pi/360 else 0
    torque = .8*ae-.160*dt+friction
    want = [-(jl[i]*force+jt[i]*torque)/.0316 for i in [1,0]]
    guard = r['q2']+2.458<m.pi/36 or r['q3']+6.421>-m.pi/36
    if guard:
        want = [min(2,.6+10*(m.pi/36-(r['q2']+2.458))) if r['q2']+2.458<m.pi/36 else 0,
                -min(2,.6+10*(r['q3']+6.421+m.pi/36)) if r['q3']+6.421>-m.pi/36 else 0]
    r.update(want2=round(max(-4,min(4,want[0])),3), want3=round(max(-4,min(4,want[1])),3),
             dl=round(dl*1000,1),guard=guard)
print('t L mm  dL mm/s | ID2 command / desired | ID3 command / desired | guard')
for r in rows:
    if 28.58<=r['t']<=29.64 or 31.58<=r['t']<=32.36:
        print(r['t'],r['L'],r['dl'],r['v2'],r['want2'],r['v3'],r['want3'],r['guard'])
print('peak', max(rows,key=lambda r:r['L']))
print('guard samples',sum(r['guard'] for r in rows))
