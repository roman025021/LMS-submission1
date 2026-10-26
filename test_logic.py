"""Run: python3 test_logic.py   (Task 1 examples + Task 9 sample sensor inputs)"""
from line_logic import *

print("=== Task 1.1 compute_centroid ===")
assert compute_centroid([0, 0, 1, 0, 0]) == 0.0
print("centred   ->", compute_centroid([0, 0, 1, 0, 0]))
print("right-ish ->", round(compute_centroid([0, 0, .4, 1, .2]), 3))
c = compute_centroid([1, .5, 0, 0, 0]); assert -1.7 < c < -1.6; print("far-left  ->", round(c, 3))
assert compute_centroid([0, 0, 0, 0, 0]) is None; print("all zero  -> None (no line)")
assert compute_centroid([-1, 0, 1, 0, 0]) == 0.0; print("negative input clipped to 0")

print("=== Task 1.2 clamp_motor ===")
assert (clamp_motor(300, 255), clamp_motor(-300, 255), clamp_motor(120, 255), clamp_motor(-7, 255)) == (255, -255, 120, -7)
assert clamp_motor(500, -200) == 200
print("clamp(300,255)=255 clamp(-300,255)=-255 clamp(120,255)=120 clamp(-7,255)=-7")

print("=== Calibration ===")
cal = Calib(); cal.update([300, 310, 290, 305, 295]); cal.update([3900, 3850, 3950, 3800, 3920])
print("raw 3900 ->", normalise(3900, cal.mn[0], cal.mx[0]), "| raw 300 ->", normalise(300, cal.mn[0], cal.mx[0]),
      "| dead sensor ->", normalise(1000, 1000, 1100))

print("=== Task 9: sample sensor sequence (dt = 10 ms) ===")
frames = [[0, 0, 1, 0, 0], [0, 0, .6, .8, 0], [0, 0, .2, .9, .5], [0, 0, 0, .7, .7],
          [0, 0, .2, .9, .1], [.7, .7, 0, 0, 0], [0, 0, 1, 0, 0]]
lf, t = LineFollower(), 0
print(" t_ms   err       u    L    R  state")
for f in frames:
    o = lf.step(f, 0.01, t); print(f"{t:5d} {o.err:5.2f} {o.u:7.1f} {o.left:4d} {o.right:4d}  {STATE_NAME[o.state]}"); t += 10
assert LineFollower().step([0, 0, 0, 1, 0], .01, 0).left > LineFollower().step([0, 0, 0, 1, 0], .01, 0).right   # line right -> left faster
o = LineFollower().step([0, 1, 0, 0, 0], .01, 0); assert o.left < o.right

print("--- 50 ms bump drop-out then line returns: must COAST, not stop ---")
b = LineFollower(); b.step([0, .2, .9, 0, 0], .01, 0)
for i in range(1, 6): o = b.step([0] * 5, .01, i * 10)
print("after 50 ms lost:", STATE_NAME[o.state], o.left, o.right); assert o.state == COAST
assert b.step([0, .2, .9, 0, 0], .01, 60).state == FOLLOW; print("line back: FOLLOW")

print("--- sustained loss, line last seen on the RIGHT ---")
b = LineFollower(); b.step([0, 0, 0, .5, 1], .01, 0); prev = None
for i in range(1, 201):
    o = b.step([0] * 5, .01, i * 10)
    if o.state != prev: print(f"t={i*10:4d} ms -> {STATE_NAME[o.state]} L={o.left:4d} R={o.right:4d}"); prev = o.state
assert prev == STOP
print("ALL TESTS PASSED")
