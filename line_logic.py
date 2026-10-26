"""line_logic.py -- Python reference implementation of Tasks 1, 3 and 9 logic.
Identical algorithm to src/line_follower/line_logic.h (the ESP32 C++ port); test and tune here on a PC."""
from dataclasses import dataclass
from math import fabs

N = 5
W = [-2.0, -1.0, 0.0, 1.0, 2.0]          # sensor position weights (Appendix A)


# ------------------------------------------------------------ Task 1.1
def compute_centroid(s, eps=1e-3):
    """Weighted centroid sum(w*s)/sum(s). Returns None when there is no signal (all-zero case)."""
    vals = [v if v > 0 else 0.0 for v in s]          # spec says non-negative; clip defensively
    den = sum(vals)
    if den <= eps:
        return None                                   # never divide by zero
    return sum(w * v for w, v in zip(W, vals)) / den


# ------------------------------------------------------------ Task 1.2
def clamp_motor(cmd, max_mag):
    """Clamp to [-|max|, +|max|]; sign preserved because only the magnitude is limited."""
    m = abs(max_mag)
    return max(-m, min(m, cmd))


# ------------------------------------------------------------ Calibration
class Calib:
    def __init__(self):
        self.mn = [4095] * N
        self.mx = [0] * N

    def update(self, raw):
        for i, r in enumerate(raw):
            self.mn[i] = min(self.mn[i], r)
            self.mx[i] = max(self.mx[i], r)


def normalise(raw, mn, mx, min_span=200):
    """raw ADC -> 0..1 (1 = black). A tiny span means a dead/unplugged sensor -> 0."""
    span = mx - mn
    if span < min_span:
        return 0.0
    return max(0.0, min(1.0, (raw - mn) / span))


def denoise(n, floor=0.12):
    """Remove the white-surface noise floor so 'white' sensors don't bias the centroid."""
    return max(0.0, (n - floor) / (1.0 - floor))


# ------------------------------------------------------------ PID
class PID:
    def __init__(self, kp=0, ki=0, kd=0, i_limit=30, i_gate=1.0, d_alpha=0.35, u_limit=160):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.i_gate, self.d_alpha, self.u_limit = i_limit, i_gate, d_alpha, u_limit
        self.reset()

    def reset(self):
        self.integ = self.prev = self.d_filt = 0.0
        self.first = True

    def update(self, err, dt):
        dt = max(dt, 1e-4)
        if self.first:                                  # no derivative kick on first sample
            self.prev, self.first = err, False
        if abs(err) < self.i_gate and self.ki > 0:      # conditional integration (anti-windup)
            self.integ += err * dt
            lim = self.i_limit / self.ki                # clamp the integral TERM
            self.integ = max(-lim, min(lim, self.integ))
        raw_d = (err - self.prev) / dt
        self.d_filt += self.d_alpha * (raw_d - self.d_filt)   # low-pass the derivative
        self.prev = err
        u = self.kp * err + self.ki * self.integ + self.kd * self.d_filt
        return max(-self.u_limit, min(self.u_limit, u))       # one bad sample can't slam the motors


# ------------------------------------------------------------ Controller + lost-line FSM
FOLLOW, COAST, SEARCH, STOP = 0, 1, 2, 3
STATE_NAME = ["FOLLOW", "COAST ", "SEARCH", "STOP  "]


@dataclass
class Params:
    kp: float = 60.0
    ki: float = 0.4
    kd: float = 12.0
    base_speed: int = 150
    slow_gain: int = 35          # speed reduction per unit |error|
    max_cmd: int = 255
    search_spd: int = 90
    lost_sum: float = 0.25
    coast_ms: int = 120
    search_ms: int = 1200


@dataclass
class Output:
    left: int = 0
    right: int = 0
    state: int = FOLLOW
    err: float = 0.0
    u: float = 0.0
    valid: bool = False


class LineFollower:
    def __init__(self, p=None):
        self.p = p or Params()
        self.pid = PID(self.p.kp, self.p.ki, self.p.kd)
        self.reset()

    def reset(self):
        self.pid.reset()
        self.last_err, self.last_l, self.last_r = 0.0, 0, 0
        self.lost_since, self.lost, self.stopped = 0, False, False

    def step(self, s, dt, now_ms):
        p, o = self.p, Output()
        if self.stopped:                                   # latched safe stop until reset()
            o.state = STOP
            return o
        pos = compute_centroid(s)
        if pos is not None and sum(s) >= p.lost_sum:       # line visible
            self.lost = False
            u = self.pid.update(pos, dt)                   # +err = line on the RIGHT -> turn right
            base = p.base_speed - int(p.slow_gain * fabs(pos))
            o.left = clamp_motor(base + int(u), p.max_cmd)     # left faster, right slower
            o.right = clamp_motor(base - int(u), p.max_cmd)
            o.err, o.u, o.valid, o.state = pos, u, True, FOLLOW
            self.last_err, self.last_l, self.last_r = pos, o.left, o.right
            return o
        if not self.lost:
            self.lost, self.lost_since = True, now_ms
        dur = now_ms - self.lost_since
        o.err = self.last_err
        if dur < p.coast_ms:                               # 1) bridge: hold last good command
            o.left, o.right, o.state = self.last_l, self.last_r, COAST
        elif dur < p.search_ms:                            # 2) pivot toward side line vanished
            self.pid.reset()
            sgn = 1 if self.last_err >= 0 else -1
            o.left = clamp_motor(sgn * p.search_spd, p.max_cmd)
            o.right = clamp_motor(-sgn * p.search_spd, p.max_cmd)
            o.state = SEARCH
        else:                                              # 3) give up: latched safe stop
            self.stopped, o.state = True, STOP
        return o
