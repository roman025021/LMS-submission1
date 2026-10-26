// line_logic.h -- hardware-independent logic (runs on ESP32 AND on a PC for testing)
// Covers Task 1 (centroid + clamp), Task 3 (PID) and Task 9 (lost-line handling).
#pragma once
#include <stdint.h>
#include <math.h>

namespace lms {

static const int N = 5;
static const float W[N] = {-2.f, -1.f, 0.f, 1.f, 2.f};   // sensor position weights (Appendix A)

// ---------------------------------------------------------------- Task 1.1
// Weighted centroid: pos = sum(w_i * s_i) / sum(s_i).
// Returns false (and leaves pos untouched) when total signal ~ 0 -> "no line seen".
// Negative inputs are treated as 0 (spec says non-negative; this is defensive).
inline bool computeCentroid(const float s[N], float &pos, float eps = 1e-3f) {
  float num = 0.f, den = 0.f;
  for (int i = 0; i < N; i++) {
    float v = s[i] > 0.f ? s[i] : 0.f;
    num += W[i] * v;
    den += v;
  }
  if (den <= eps) return false;       // explicit all-zero case: never divide by zero
  pos = num / den;
  return true;
}

// ---------------------------------------------------------------- Task 1.2
// Clamp a signed motor command to +/-maxMag. Sign is preserved by construction.
inline int clampMotor(int cmd, int maxMag) {
  if (maxMag < 0) maxMag = -maxMag;   // tolerate a negative "max"
  if (cmd >  maxMag) return  maxMag;
  if (cmd < -maxMag) return -maxMag;
  return cmd;
}

// ---------------------------------------------------------------- Calibration
struct Calib { int mn[N]; int mx[N]; };
inline void calibReset(Calib &c) { for (int i = 0; i < N; i++) { c.mn[i] = 4095; c.mx[i] = 0; } }
inline void calibUpdate(Calib &c, const int raw[N]) {
  for (int i = 0; i < N; i++) {
    if (raw[i] < c.mn[i]) c.mn[i] = raw[i];
    if (raw[i] > c.mx[i]) c.mx[i] = raw[i];
  }
}
// raw ADC -> 0..1 (1 = black line). A sensor whose span is tiny is dead/unplugged -> 0.
inline float normalise(int raw, int mn, int mx, int minSpan = 200) {
  int span = mx - mn;
  if (span < minSpan) return 0.f;
  float v = (float)(raw - mn) / (float)span;
  return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
}
// Remove the white-surface noise floor, otherwise 4 "white" sensors bias the centroid toward 0.
inline float denoise(float n, float floorV = 0.12f) {
  float v = (n - floorV) / (1.f - floorV);
  return v > 0.f ? v : 0.f;
}

// ---------------------------------------------------------------- PID
struct PID {
  float kp, ki, kd;
  float iLimit;      // clamp on the integral TERM (in motor counts)
  float iGate;       // only integrate while |err| < iGate (stops windup in sharp turns)
  float dAlpha;      // 0..1 low-pass factor on the derivative (1 = no filtering)
  float uLimit;      // clamp on total PID output (stops D-kick from a single bad sample, e.g. a bump)
  float integ, prevErr, dFilt; bool first;

  PID(float p = 0, float i = 0, float d = 0, float il = 30, float ig = 1.0f, float da = 0.35f, float ul = 160.f)
    : kp(p), ki(i), kd(d), iLimit(il), iGate(ig), dAlpha(da), uLimit(ul), integ(0), prevErr(0), dFilt(0), first(true) {}

  void reset() { integ = 0; prevErr = 0; dFilt = 0; first = true; }

  float update(float err, float dt) {
    if (dt <= 1e-4f) dt = 1e-4f;
    if (first) { prevErr = err; first = false; }          // no derivative kick on first sample
    if (fabsf(err) < iGate && ki > 0.f) {                 // conditional integration
      integ += err * dt;
      float lim = iLimit / ki;                            // clamp integral term, not the raw sum
      if (integ >  lim) integ =  lim;
      if (integ < -lim) integ = -lim;
    }
    float rawD = (err - prevErr) / dt;
    dFilt += dAlpha * (rawD - dFilt);                     // 1st-order low-pass on D
    prevErr = err;
    float u = kp * err + ki * integ + kd * dFilt;
    if (u >  uLimit) u =  uLimit;
    if (u < -uLimit) u = -uLimit;
    return u;
  }
};

// ---------------------------------------------------------------- Controller + lost-line FSM
enum State { ST_FOLLOW = 0, ST_COAST = 1, ST_SEARCH = 2, ST_STOP = 3 };

struct Params {
  float kp = 60.f, ki = 0.4f, kd = 12.f;  // chosen from src/python/sim.py sweep (docs/theory_notes.pdf, Task 3)
  int   baseSpeed  = 150;     // PWM counts (0..255) on straights
  int   slowGain   = 35;      // speed reduction per unit |error| (slow down in turns)
  int   maxCmd     = 255;
  int   searchSpd  = 90;      // pivot speed while searching
  float lostSum    = 0.25f;   // sum of denoised sensors below this = line lost
  uint32_t coastMs = 120;     // bridge short drop-outs (bumps / gaps) with last command
  uint32_t searchMs= 1200;    // then pivot toward last-seen side; after this, STOP
};

struct Output { int left, right; State st; float err; float u; bool valid; };

class LineFollower {
 public:
  Params p; PID pid;
  explicit LineFollower(const Params &pp = Params())
    : p(pp), pid(pp.kp, pp.ki, pp.kd, 30.f, 1.0f, 0.35f) { reset(); }

  void reset() {
    pid.reset(); lastErr = 0; lastL = lastR = 0; lostSince = 0; lost = false; stopped = false;
  }

  // s[] = normalised + denoised sensor values (0..1), dt in seconds, now in ms.
  Output step(const float s[N], float dt, uint32_t nowMs) {
    Output o{0, 0, ST_FOLLOW, 0.f, 0.f, false};
    float sum = 0; for (int i = 0; i < N; i++) sum += s[i];
    float pos = 0;
    bool valid = computeCentroid(s, pos) && sum >= p.lostSum;

    if (stopped) { o.st = ST_STOP; return o; }          // latched safe stop until reset()

    if (valid) {
      lost = false;
      float u = pid.update(pos, dt);                    // +err = line on the RIGHT -> turn right
      int base = p.baseSpeed - (int)(p.slowGain * fabsf(pos));
      o.left  = clampMotor(base + (int)u, p.maxCmd);    // turn right: left faster, right slower
      o.right = clampMotor(base - (int)u, p.maxCmd);
      o.err = pos; o.u = u; o.valid = true; o.st = ST_FOLLOW;
      lastErr = pos; lastL = o.left; lastR = o.right;
      return o;
    }

    if (!lost) { lost = true; lostSince = nowMs; }
    uint32_t dur = nowMs - lostSince;
    o.err = lastErr;
    if (dur < p.coastMs) {                              // 1) bridge: hold last good command
      o.left = lastL; o.right = lastR; o.st = ST_COAST;
    } else if (dur < p.searchMs) {                      // 2) pivot toward side where line vanished
      pid.reset();
      int sgn = (lastErr >= 0) ? 1 : -1;
      o.left  = clampMotor( sgn * p.searchSpd, p.maxCmd);
      o.right = clampMotor(-sgn * p.searchSpd, p.maxCmd);
      o.st = ST_SEARCH;
    } else {                                            // 3) give up: safe stop (latched)
      stopped = true; o.st = ST_STOP;
    }
    return o;
  }

 private:
  float lastErr; int lastL, lastR; uint32_t lostSince; bool lost, stopped;
};

}  // namespace lms
