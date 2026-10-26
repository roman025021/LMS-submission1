"""Closed-loop kinematic sim used to choose gains. python3 sim.py
Diff-drive robot, array 3 cm ahead of axle, pitch 14 mm, 80 ms motor lag, 10 ms loop."""
import math
from line_logic import *

def run(kp, ki, kd, y0=0.02, curve_k=0.0):
    lf = LineFollower(Params(kp=kp, ki=ki, kd=kd))
    dt, pitch, Ls, Wb, kv, tau, sigma = 0.01, 0.014, 0.03, 0.09, 0.52 / 255, 0.08, 0.009
    x, y, th, vl, vr = 0.0, y0, 0.0, 0.0, 0.0
    sq, n, settle, zc, pe = 0.0, 0, -1.0, 0, 0.0
    lost = False
    for k in range(600):
        sx, sy = x + Ls * math.cos(th), y + Ls * math.sin(th)
        line_y = curve_k * sx * sx * 0.5
        d = (sy - line_y) / math.cos(th)                     # +ve = line on the right
        s = [denoise(math.exp(-0.5 * ((d - w * pitch) / sigma) ** 2)) for w in W]
        o = lf.step(s, dt, k * 10)
        lost |= o.state in (SEARCH, STOP)
        vl += (o.left * kv - vl) * dt / tau
        vr += (o.right * kv - vr) * dt / tau
        v, w = (vl + vr) / 2, (vr - vl) / Wb
        x += v * math.cos(th) * dt; y += v * math.sin(th) * dt; th += w * dt
        e = (sy - line_y) * 1000                              # mm
        if abs(e) > 0.5:
            if pe * e < 0: zc += 1
            pe = e
        if k >= 300: sq += e * e; n += 1
        if abs(e) < 2 and settle < 0: settle = k * 0.01
        if abs(e) > 3: settle = -1
    return math.sqrt(sq / n), settle, zc, lost

print("gains                 rms(3-6s)  settle  zero-crossings")
for g in [(60, .4, 12), (60, 0, 0), (60, 0, 12), (32, 0, 0), (80, 0, 0), (150, 0, 0), (250, 0, 0), (80, 0, 10), (150, 0, 25), (250, 0, 25), (250, 0, 50)]:
    r, s_, z, l = run(*g); print(f"kp={g[0]:<4} ki={g[1]:<3} kd={g[2]:<3}  {r:6.2f}mm  {s_:5.2f}s  {z}  lost={int(l)}")
print("\n-- constant-curvature arc (R = 0.5 m), steady error --")
for g in [(60, 0, 12), (60, .4, 12), (60, 2, 12)]:
    r, *_ = run(*g, y0=0.0, curve_k=2.0); print(f"kp={g[0]} ki={g[1]} kd={g[2]}  rms {r:.2f} mm")
