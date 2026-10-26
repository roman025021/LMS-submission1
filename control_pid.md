# PID for line following (Task 3; implemented in Task 9)

## 1. What each term does
Error `e` = line position in sensor-pitch units (0 = centred, +2 = line under the rightmost sensor). Steering correction `u = Kp*e + Ki*Int(e) + Kd*de/dt`.

* **P** - steers proportionally to *where* the line is now. Too low: sluggish, loses the line in curves. Too high: overshoots and weaves (the robot is a delayed system: sensor is ahead of the axle, motors lag).
* **I** - accumulates past error; removes *steady* bias (e.g. constant-curvature arc, one motor slightly weaker). Risk: windup in sharp turns / after a lost line, which causes a late violent swing.
* **D** - reacts to *how fast* the error changes; acts as damping, predicts the crossing so the robot arrives at the line without overshooting. Risk: amplifies noise and bump spikes.

## 2. Configuration chosen: PD-dominant, tiny gated I (Kp=60, Ki=0.4, Kd=12)
Lateral position is already an integral of heading, which is an integral of steering, so a P term alone has no steady error on a straight line; the main enemy is **oscillation**, which D fixes. I is only a small trim (clamped to 30 counts, only integrates when |e|<1, reset on line loss). Simulation (`src/test/sim.cpp`, diff-drive model, 0.03 m sensor lead, 80 ms motor lag, start 20 mm off-line):

| gains (Kp,Ki,Kd) | steady RMS err | settle to 2 mm | zero-crossings (weave) |
|---|---|---|---|
| 32, 0, 0 (P only) | 3.05 mm | 5.22 s | 11 |
| 60, 0, 0 (P only) | 1.30 mm | 2.92 s | 15 |
| **60, 0.4, 12 (chosen)** | **0.16 mm** | **0.36 s** | **0** |
| 60, 0, 12 (PD) | 0.17 mm | 0.36 s | 0 |
| 250, 0, 25 | 0.04 mm | 0.19 s | 0 (too aggressive for real noise/saturation) |
| 250, 0, 50 | 0.02 mm | 0.43 s | 0 (too much D: slower) |

On a 0.5 m-radius arc Ki=0 -> 0.23 mm, Ki=0.4 -> 0.22 mm, Ki=2 -> 0.16 mm: I helps little, so it stays small. *Caveat:* an idealised model; real tuning on the track is required (section 5).

## 3. Sensors -> error -> motors
1. `analogRead` (12-bit, 2x oversampled) -> min/max calibration -> `n = (raw-min)/(max-min)` in [0,1] (1 = black).
2. Subtract a 0.12 noise floor and clip at 0 (otherwise four "white" sensors reading ~5 % bias the centroid toward 0).
3. Centroid `e = sum(w_i s_i)/sum(s_i)`, `w = [-2,-1,0,1,2]`. All-zero -> "no line" (never divide by zero).
4. `u = PID(e)` (clamped to +/-160 so one bump spike cannot slam the motors).
5. `base = 150 - 35*|e|` (slow down when off-centre/in turns).
6. `left = base + u`, `right = base - u`, each clamped to +/-255 with sign preserved. Positive e = line on right -> left wheel faster -> robot turns right. (Negative PWM lets the inner wheel reverse for tight turns.)

```
loop every 10 ms:
  s[] = denoise(normalise(analogRead x5))
  if line_visible(s):  e = centroid(s); u = PID(e, dt)
                       L = base(e)+u ; R = base(e)-u ; remember last e, L, R
  else:                see lost-line ladder below
  drive(clamp(L), clamp(R))
```

## 4. Lost line and oscillation
**Lost line (sum of sensors < 0.25), escalating:**
1. 0-120 ms **COAST**: repeat last good command. Bumps/gaps cause 10-50 ms drop-outs (Task 7), so stopping here would make the robot twitch.
2. 120-1200 ms **SEARCH**: reset PID, pivot in place at 90 PWM toward the side where the line was last seen (`sign(last e)`).
3. >1200 ms **STOP**: motors off, latched until `reset()`. Safe failure beats driving off the table.
Wide black bar / junction (all five dark) gives a centroid of 0 -> drives straight, which is the right default.

**Oscillation response (in order):** lower base speed; lower Kp by ~20 %; raise Kd until the weave dies (stop when motors buzz = noise amplification; then raise `dAlpha` filtering); check loop period is constant (print dt); check motor dead-zone (<~60 PWM motors stall) and L/R asymmetry; check sensor height (too high -> blurred centroid -> low gain).

## 5. Real-robot tuning procedure
Ki=Kd=0, base speed low (100). Raise Kp until it sustains a steady weave (Ku, period Pu). Set Kp = 0.5-0.6 Ku, Kd ~ Kp*Pu/8 (Ziegler-Nichols, then back off), Ki = 0 then small. Raise speed in steps, retune. Log CSV over serial and plot.
