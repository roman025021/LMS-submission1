// line_follower.ino -- ESP32-WROOM-32, Arduino framework. Task 9 (uses Tasks 1 and 3 logic).
// Build: Arduino-ESP32 core 2.0.x or 3.x. Hardware per Appendix A with the two documented deviations below.
#include <Arduino.h>
#include "line_logic.h"
using namespace lms;

// ------------------------------------------------------------------ PIN MAP
// Appendix A, with two engineering corrections (see docs/theory_notes.pdf, Task 6):
//  * IN4 on GPIO11: GPIO6-11 are wired to the module's SPI flash on ESP32-WROOM-32 -> using it crashes the chip.
//    Remapped to GPIO4.
//  * IR_S5 on GPIO25 is ADC2_CH8 (not ADC1). ADC2 cannot be read while Wi-Fi is on (Task 7) -> use GPIO39 (ADC1_CH3).
// Define APPENDIX_STRICT_PINS to compile with the literal Appendix mapping.
#ifdef APPENDIX_STRICT_PINS
  #define PIN_S5  25
  #define PIN_IN4 11
#else
  #define PIN_S5  39
  #define PIN_IN4 4
#endif
static const uint8_t IR_PIN[N] = {34, 35, 32, 33, PIN_S5};
static const uint8_t ENA = 14, IN1 = 27, IN2 = 26;       // left motor
static const uint8_t ENB = 13, IN3 = 12, IN4 = PIN_IN4;  // right motor (GPIO12 is a boot-strap pin: fit 10k pull-down)

// ------------------------------------------------------------------ CONFIG
static const uint32_t LOOP_US   = 10000;   // 100 Hz. 200 Hz x ~60 char telemetry would exceed 115200 baud.
static const int      PWM_HZ    = 5000;    // L298N bipolar outputs switch slowly; 5 kHz keeps heating low
static const bool     LEFT_INV  = false, RIGHT_INV = false;   // flip if a motor spins backwards
static const int      CAL_SPD   = 90;      // pivot speed during calibration
static const uint32_t CAL_MS    = 3000;

Calib cal;
LineFollower lf;                           // default Params = tuned gains from line_logic.h

// ------------------------------------------------------------------ PWM shim (core 2.x vs 3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static void pwmBegin()                 { ledcAttach(ENA, PWM_HZ, 8); ledcAttach(ENB, PWM_HZ, 8); }
  static void pwmPut(uint8_t pin, int d) { ledcWrite(pin, d); }
#else
  static void pwmBegin()                 { ledcSetup(0, PWM_HZ, 8); ledcAttachPin(ENA, 0); ledcSetup(1, PWM_HZ, 8); ledcAttachPin(ENB, 1); }
  static void pwmPut(uint8_t pin, int d) { ledcWrite(pin == ENA ? 0 : 1, d); }
#endif

// cmd in [-255,255]; sign = direction, magnitude = PWM duty on the L298N enable pin
static void drive(uint8_t inA, uint8_t inB, uint8_t en, int cmd, bool inv) {
  if (inv) cmd = -cmd;
  cmd = clampMotor(cmd, 255);
  digitalWrite(inA, cmd > 0); digitalWrite(inB, cmd < 0);     // 0 -> both LOW = coast
  pwmPut(en, abs(cmd));
}
static void setMotors(int l, int r) { drive(IN1, IN2, ENA, l, LEFT_INV); drive(IN3, IN4, ENB, r, RIGHT_INV); }

// ------------------------------------------------------------------ SENSORS
static void readRaw(int raw[N]) {
  for (int i = 0; i < N; i++) raw[i] = (analogRead(IR_PIN[i]) + analogRead(IR_PIN[i])) / 2;  // 2x oversample
}
static void readNorm(float s[N], int raw[N]) {
  readRaw(raw);
  for (int i = 0; i < N; i++) s[i] = denoise(normalise(raw[i], cal.mn[i], cal.mx[i]));
}

static void calibrate() {
  calibReset(cal);
  uint32_t t0 = millis(); int raw[N];
  while (millis() - t0 < CAL_MS) {                       // wiggle left/right so every sensor sees line AND floor
    int dir = (((millis() - t0) / 500) % 2) ? 1 : -1;
    setMotors(dir * CAL_SPD, -dir * CAL_SPD);
    readRaw(raw); calibUpdate(cal, raw); delay(5);
  }
  setMotors(0, 0);
  for (int i = 0; i < N; i++)
    Serial.printf("# cal S%d min=%d max=%d%s\n", i + 1, cal.mn[i], cal.mx[i], (cal.mx[i]-cal.mn[i] < 200) ? "  <-- WEAK/DEAD SENSOR" : "");
}

// ------------------------------------------------------------------ ARDUINO
void setup() {
  Serial.begin(115200);
  const uint8_t outs[] = {IN1, IN2, IN3, IN4, ENA, ENB};
  for (uint8_t p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }   // motors off before anything else
  analogReadResolution(12);                                              // 0..4095
  analogSetAttenuation(ADC_11db);                                        // full 0..3.3 V range
  pwmBegin();
  Serial.println("# t_us,raw1..raw5,pos,u,L,R,state");
  delay(2000);                                                           // time to place robot on the line
  calibrate();
}

void loop() {
  static uint32_t next = micros(), last = micros();
  while ((int32_t)(micros() - next) < 0) { }                             // fixed-rate loop
  next += LOOP_US;
  uint32_t now = micros(); float dt = (now - last) * 1e-6f; last = now;

  int raw[N]; float s[N];
  readNorm(s, raw);
  Output o = lf.step(s, dt, millis());
  setMotors(o.left, o.right);                                            // ST_STOP gives 0,0

  // telemetry every iteration (spec). Comment out / decimate if loop time drifts.
  Serial.printf("%lu,%d,%d,%d,%d,%d,%.3f,%.1f,%d,%d,%d\n", (unsigned long)now,
                raw[0], raw[1], raw[2], raw[3], raw[4], o.err, o.u, o.left, o.right, (int)o.st);
}
