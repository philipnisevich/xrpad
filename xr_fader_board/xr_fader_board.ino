// XR18 fader board — a second ESP32-S3 that runs up to 8 motorized faders
// for the xr_console main board. It has no Wi-Fi: it only moves faders to
// the positions the main board asks for, and reports where they are and
// which ones a hand is on.
//
// Each fader follows a glide: a setpoint that speeds up and eases in
// smoothly, which the motor tracks (feed-forward plus correction, slow-decay
// PWM). Streams of targets from the app or encoder play back as one steady
// motion. A hand on a fader stops its motor at once. At power-up all faders
// calibrate together, then play a short wave show.
//
// Wiring (logic is 3V3; motor drivers on their own 5 V supply, grounds shared):
//   Fader n wiper (n = 1-8)    -> G1..G8   (top end 3V3, bottom end GND)
//   Fader n motor driver IN1   -> G9..G16
//   Fader n motor driver IN2   -> G17, G18, G21, G38, G39, G40, G41, G42
//   Fader n motor outputs      -> TC1508A MOTOR-A/B (two faders per driver)
//   Fader n touch track        -> MPR121 ELE0-ELE3 (faders 1-4), ELE5-ELE8 (faders 5-8)
//   MPR121                     SDA -> G47, SCL -> G48, VCC -> 3V3, GND, ADDR -> GND
//   Link to main board         TX G45 -> main G14, RX G46 <- main G13, GND
//   Pull-downs (10k to GND)    G45 (else the flash may boot at 1.8 V) and
//                              G39 (else fader 5 runs up during boot)
//   TCA9548A multiplexer       SDA -> G47, SCL -> G48 (the MPR121's bus), VIN 3V3,
//                              GND, A0-A2 -> GND (0x70), RST -> 3V3
//   Fader n display            0.96" SSD1306 OLED (0x3C): SDA -> TCA SD(n-1),
//                              SCL -> TCA SC(n-1), VCC 3V3, GND
//                              (no TCA9548A: one OLED straight on G47/G48 = fader 1)
//   Unplug the G46 wire to flash this board: held high it blocks download mode.
//   (G43/G44 aren't on the LCKFB board's headers.)
//
// Without an MPR121 (NATIVE_TOUCH): faders 1-4 only, touch on the ESP32's
// own pins G5..G8, link on TX G48 / RX G47. For 8 faders that way, use two
// of these boards with this same firmware: the second one's link goes to
// the main board's G1 (to its G47) and G2 (from its G48), and the main board
// treats its faders 1-4 as bank faders 5-8.
//
// Protocol (921600 baud, text lines):
//   main -> board  "T <n> <pos>"  move fader n (0-7) to pos 0-1000; pos < 0 = no target
//                  "C"            recalibrate all faders
//                  "?"            ask for the hello line
//                  "N <n> <on> <solo> <label>|<name>"  what fader n is assigned to, for
//                                 its display (on 0 = muted; empty label = nothing assigned)
//   board -> main  "H <faders> <touch sensor>"     hello (at boot and on "?")
//                  "S <touchmask> <p0> ... <p7>"   every 20 ms, positions 0-1000

#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <U8g2lib.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

const int MAX_FADERS = 8;
const int FADERS = 8;               // how many are fitted (1 for testing with a single fader)
const bool NATIVE_TOUCH = false;    // true: touch on G5-G8 (4 faders max); false: MPR121
const uint8_t TOUCH_PIN[4] = {5, 6, 7, 8};
const uint8_t MPR_ELECTRODE[MAX_FADERS] = {0, 1, 2, 3, 5, 6, 7, 8};  // ELE4 is skipped
static_assert(!NATIVE_TOUCH || FADERS <= 4, "native touch uses fader 5-8's wiper pins: 4 faders max");

const uint8_t WIPER[MAX_FADERS] = {1, 2, 3, 4, 5, 6, 7, 8};
const uint8_t IN1[MAX_FADERS]   = {9, 10, 11, 12, 13, 14, 15, 16};   // PWM
const uint8_t IN2[MAX_FADERS]   = {17, 18, 21, 38, 39, 40, 41, 42};  // direction
const bool MOTOR_REVERSE[MAX_FADERS] = {false, false, false, false, false, false, false, false};
const bool POT_REVERSE[MAX_FADERS]   = {false, false, false, false, false, false, false, false};

const uint8_t I2C_SDA = 47, I2C_SCL = 48;
const uint8_t LINK_TX = NATIVE_TOUCH ? 48 : 45;  // G48 also lights the onboard LED
const uint8_t LINK_RX = NATIVE_TOUCH ? 47 : 46;
const uint32_t LINK_BAUD = 921600;

const uint32_t MOTOR_PWM_HZ = 20000;  // above hearing
const uint8_t  MOTOR_PWM_BITS = 10;
const int      PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
// Measured (USB "V n"): speed = MOTOR_K * (power - MOTOR_D0) travel/s, with
// these faders moving from about 25-30% power either way.
const float    MOTOR_K = 18.0f;
const float    MOTOR_D0 = 0.20f;          // power that just balances friction
const float    MOTOR_KP = 6.0f;           // power per unit of lag behind the glide
const float    MOTOR_KD = 0.08f;          // power per travel/s faster than the glide (damping)
const float    MOTOR_FULL_FRICTION_VEL = 0.5f; // glide speed (travel/s) from which friction is fully offset
const float    MOVE_VMAX = 6.0f;          // glide speed limit, travel/s
const float    MOVE_AMAX = 60.0f;         // glide acceleration, travel/s^2
const float    MOTOR_TOL = 0.008f;
const float    MOTOR_STILL_VEL = 0.3f;    // travel/s: slower than this counts as stopped
const uint32_t MOTOR_HOLD_MS = 80;        // brake this long at the end of a move
const uint32_t MOTOR_SETTLE_CHECK_MS = 80; // after that, check it stayed put
const uint8_t  MOTOR_CORRECTIONS = 3;     // gentle nudges back if it drifted off
const uint32_t MOTOR_STALL_MS = 25;       // not moving this long while it should: push harder
const uint32_t MOTOR_KICK_MS = 150;       // ...ramping to full power over this long
const float    MOTOR_HELD_LAG = 0.2f;     // this far behind the glide...
const uint32_t MOTOR_HELD_MS = 250;       // ...for this long = held or blocked: let go
const uint32_t MOTOR_TIMEOUT_MS = 800;    // glide finished but still not there: give up
const uint32_t FOLLOW_MS = 300;           // targets closer together than this = a stream
const uint32_t STREAM_DELAY_MS = 80;      // streams play back this far behind (a jitter buffer)
const float    STREAM_PULL = 25.0f;       // how firmly the glide holds to the stream's line, 1/s

const uint32_t CAL_DRIVE_MS = 500;
const float    CAL_END_GAP = 0.015f;    // -inf/+10 dB sit this far in from the end stops
const int      CAL_MIN_SPAN = 1800;     // raw ADC counts; less = calibration failed

const uint32_t TOUCH_READ_MS = 15;
const uint32_t TOUCH_HOLD_MS = 150;     // stay "touched" briefly after letting go
const float    TOUCH_ON = 1.00f;        // native touch: this far above baseline = touched (a finger reads 3-10x)
const float    TOUCH_OFF = 0.50f;       // ...and below this = released
const int      TOUCH_CONFIRM = 3;       // readings in a row over TOUCH_ON before it counts
const uint32_t REPORT_MS = 20;

// A hand is also detected by the fader moving while its motor isn't driving
// it, which covers a touch pad that isn't working or isn't fitted.
const float    NOTOUCH_MOVE = 0.03f;
const uint32_t NOTOUCH_STILL_MS = 400;
const uint32_t MOTION_SETTLE_MS = 400;  // after the motor stops, before motion counts as a hand

struct Fader {
  float minRaw = 60, maxRaw = 4000;  // calibrated ends
  float smooth = -1;                 // position 0-1, filtered
  float target = -1;                 // -1 = no target
  float sp = -1, spVel = 0;          // setpoint gliding towards the target, and its speed
  bool streaming = false;            // following a stream of targets (see glide())
  static const int HIST = 8;         // ...its recent targets and when they arrived
  float histV[HIST];
  uint32_t histT[HIST];
  uint8_t histN = 0;                 // how many are stored (newest last)
  uint16_t dbgCount = 0, dbgBunched = 0;  // DEBUG: stream timing, logged when it ends
  uint32_t dbgMin = 0, dbgMax = 0, dbgSum = 0;
  bool moving = false;
  volatile bool touched = false;     // set by loop(), acted on by the motor task
  float progressPos = 0;             // where it was when it last made progress
  uint32_t progressAt = 0, arrivedAt = 0, lagSince = 0, releasedAt = 0, brakeUntil = 0;
  float restPos = -1;                // no-sensor hand detection
  uint32_t movedAt = 0;
  float touchBase = 0;               // native touch: untouched reading
  float touchLast = 0;               // ...latest reading
  int touchCount = 0;                // ...readings in a row over the threshold
  bool padTouched = false;           // ...what the pad alone says
  bool touchOk = false;              // native touch reading looks sane (else motion only)
  uint32_t idleSince = 0;            // when the motor last stopped
  float lastRaw = 0;                 // last wiper reading (for the debug line)
  float vel = 0;                     // measured speed, travel/s (filtered)
  uint32_t checkAt = 0;              // when to check where a move really stopped
  uint8_t corrections = 0;           // settle-back nudges used by this move
  uint32_t lastTargetAt = 0;
  bool enabled = false;              // calibrated; a fader that failed is never driven
  bool flip = false;                 // motor wired the other way round (found at calibration)
};

Fader faders[MAX_FADERS];
float lastDuty[MAX_FADERS];  // DEBUG: last motor power, for the recording
Adafruit_MPR121 mpr;
bool haveMpr = false;

// ---------- motor ----------

// Each fader has one PWM channel. Driving either way, one input is held high
// and the other gets the inverted PWM, so the motor alternates between drive
// and brake (slow decay). Its speed then follows the power evenly from ~30%
// up. Drive / coast (fast decay), which the old wiring gave one direction,
// needed 70% to move at all and then lurched.
enum PinMode2 : uint8_t { PIN_LOW, PIN_HIGH, PIN_PWM };
uint8_t pinState[MAX_FADERS][2];

void setMotorPin(int n, int which, uint8_t mode) {
  uint8_t pin = which ? IN2[n] : IN1[n];
  if (pinState[n][which] == mode) return;
  pinState[n][which] = mode;
  if (mode == PIN_PWM) {
    esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + n, true, false);  // inverted
  } else {
    gpio_set_level((gpio_num_t)pin, mode == PIN_HIGH);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
  }
}

void initMotor(int n) {
  ledcAttachChannel(IN1[n], MOTOR_PWM_HZ, MOTOR_PWM_BITS, n);
  pinMode(IN2[n], OUTPUT);
  pinState[n][0] = pinState[n][1] = 255;
}

void drive(int n, float speed) {
  lastDuty[n] = speed;
  if (MOTOR_REVERSE[n] != faders[n].flip) speed = -speed;
  ledcWriteChannel(n, (uint32_t)(constrain(fabsf(speed), 0.0f, 1.0f) * PWM_MAX));
  if (speed >= 0.0f) {  // forward = IN1 high, IN2 low
    setMotorPin(n, 1, PIN_PWM);
    setMotorPin(n, 0, PIN_HIGH);
  } else {
    setMotorPin(n, 0, PIN_PWM);
    setMotorPin(n, 1, PIN_HIGH);
  }
}

void coast(int n) { lastDuty[n] = 0; setMotorPin(n, 0, PIN_LOW); setMotorPin(n, 1, PIN_LOW); }
void brake(int n) { lastDuty[n] = 0; setMotorPin(n, 0, PIN_HIGH); setMotorPin(n, 1, PIN_HIGH); }

// Median of three reads, so a single dropout from a loose wiper is ignored.
float readPosition(int n) {
  Fader &f = faders[n];
  int a = analogRead(WIPER[n]), b = analogRead(WIPER[n]), c = analogRead(WIPER[n]);
  float raw = max(min(a, b), min(max(a, b), c));
  f.lastRaw = raw;
  float pos = constrain((raw - f.minRaw) / (f.maxRaw - f.minRaw), 0.0f, 1.0f);
  return POT_REVERSE[n] ? 1.0f - pos : pos;
}

// Moves follow a glide: a setpoint (sp) that speeds up and slows down
// smoothly towards the target, and the motor tracks it. A stream of targets
// (the app's fader, the encoder, a linked partner) arrives in small steps;
// the glide also matches the stream's speed, so the fader runs through the
// steps in one smooth motion instead of stopping at each one.
void startGlide(Fader &f, uint32_t now) {
  f.moving = true;
  f.sp = f.smooth;
  f.spVel = 0;
  f.arrivedAt = 0;
  f.lagSince = 0;
  f.progressAt = now;
  f.progressPos = f.smooth;
}

void setTarget(int n, float target, uint32_t now) {
  Fader &f = faders[n];
  uint32_t gap = now - f.lastTargetAt;
  bool stream = f.target >= 0 && target >= 0 && gap < FOLLOW_MS;
  f.lastTargetAt = now;
  // Only take the target once the move can start: a target that couldn't be
  // acted on (fader not read yet, a hand on it) must not look "already done"
  // when the main board re-sends it.
  if (!f.enabled || target < 0 || f.touched || f.smooth < 0) {
    f.moving = false;
    f.target = -1;
    return;
  }
  f.target = target;
  f.checkAt = 0;
  f.corrections = 0;
  f.arrivedAt = 0;
  if (!f.moving) startGlide(f, now);
  // Keep the stream's recent targets for glide() to play back. Two arriving
  // together (the link delivers them in bursts) count as one, updated.
  if (stream) {
    if (!f.streaming) {  // the stream's first target is already stored (below)
      f.streaming = true;
      f.dbgCount = 0;
    }
    if (f.histN && gap < 5) {
      f.histV[f.histN - 1] = target;
    } else {
      if (f.histN == Fader::HIST) {
        memmove(f.histV, f.histV + 1, sizeof(float) * (Fader::HIST - 1));
        memmove(f.histT, f.histT + 1, sizeof(uint32_t) * (Fader::HIST - 1));
        f.histN--;
      }
      f.histV[f.histN] = target;
      f.histT[f.histN] = now;
      f.histN++;
    }
    if (!f.dbgCount) { f.dbgMin = gap; f.dbgMax = gap; f.dbgSum = 0; f.dbgBunched = 0; }
    f.dbgCount++;
    f.dbgMin = min(f.dbgMin, gap);
    f.dbgMax = max(f.dbgMax, gap);
    f.dbgSum += gap;
    if (gap < 5) f.dbgBunched++;
  } else {
    f.streaming = false;
    f.histN = 0;
  }
  if (!f.streaming) {  // this may be a stream's first target: remember it
    f.histV[0] = target;
    f.histT[0] = now;
    f.histN = 1;
  }
}

// Advance the setpoint by dt seconds towards the target.
void glide(Fader &f, float dt, uint32_t now) {
  // The stream has ended once its last target is fully played back.
  if (f.streaming && now - f.lastTargetAt > STREAM_DELAY_MS + 40) {
    f.streaming = false;
    if (f.dbgCount >= 3)  // DEBUG
      Serial.printf("fader %d stream: %u targets, gaps %lu/%lu/%lu ms (min/avg/max), %u bunched\n",
                    (int)(&f - faders) + 1, f.dbgCount, f.dbgMin, f.dbgSum / f.dbgCount, f.dbgMax, f.dbgBunched);
    f.dbgCount = 0;
  }
  float want;
  if (f.streaming) {
    // A stream plays back STREAM_DELAY_MS behind real time, along straight
    // lines between the targets: a late or bunched target is then still in
    // the past when it's needed, so uneven arrival can't make the fader surge
    // (a jitter buffer). The setpoint follows that line at its speed, plus a
    // pull back onto it.
    uint32_t at = now - STREAM_DELAY_MS;
    float r = f.histV[0], slope = 0.0f;
    for (int i = f.histN - 1; i >= 0; i--) {
      if ((int32_t)(at - f.histT[i]) < 0) continue;
      r = f.histV[i];
      if (i + 1 < f.histN) {
        float span = (f.histT[i + 1] - f.histT[i]) / 1000.0f;
        slope = (f.histV[i + 1] - f.histV[i]) / span;
        r += slope * (at - f.histT[i]) / 1000.0f;
      }
      break;
    }
    want = slope + STREAM_PULL * (r - f.sp);
  } else {
    // The fastest speed that still slows down smoothly onto the target.
    float e = f.target - f.sp;
    want = copysignf(sqrtf(2.0f * MOVE_AMAX * fabsf(e)), e);
  }
  want = constrain(want, -MOVE_VMAX, MOVE_VMAX);
  f.spVel += constrain(want - f.spVel, -MOVE_AMAX * dt, MOVE_AMAX * dt);
  float next = f.sp + f.spVel * dt;
  if (!f.streaming && (next - f.target) * (f.sp - f.target) <= 0.0f) { next = f.target; f.spVel = 0; }  // never past it
  f.sp = constrain(next, 0.0f, 1.0f);
}

void stopMove(Fader &f, int n) {
  coast(n);
  f.moving = false;
  f.spVel = 0;
}

void updateFader(int n, uint32_t now, float dt) {
  Fader &f = faders[n];
  float pos = readPosition(n);
  float prev = f.smooth;
  f.smooth = f.smooth < 0 ? pos : f.smooth + 0.3f * (pos - f.smooth);
  if (prev >= 0 && dt > 0) f.vel += 0.1f * ((f.smooth - prev) / dt - f.vel);

  // After stopping, check it stayed on target; nudge it back gently if not.
  if (f.brakeUntil && (now >= f.brakeUntil || f.touched || f.moving)) {
    if (!f.moving) coast(n);
    f.brakeUntil = 0;
  }
  if (!f.moving && f.checkAt && now >= f.checkAt) {
    f.checkAt = 0;
    if (f.target >= 0 && !f.touched && fabsf(f.target - f.smooth) > MOTOR_TOL * 2 &&
        f.corrections < MOTOR_CORRECTIONS) {
      f.corrections++;
      startGlide(f, now);
    }
  }
  if (!f.moving) return;
  if (f.touched) { stopMove(f, n); return; }

  glide(f, dt, now);
  float err = f.target - f.smooth;
  float lag = f.sp - f.smooth;
  bool arrived = f.sp == f.target && !f.streaming;
  if (arrived && !f.arrivedAt) f.arrivedAt = now;

  if (arrived && fabsf(err) <= MOTOR_TOL && fabsf(f.vel) < MOTOR_STILL_VEL) {
    // Hold it with the brake for a moment: let go straight away and the
    // belt's spring (strongest near the top) slides it on by a percent.
    stopMove(f, n);
    brake(n);
    f.brakeUntil = now + MOTOR_HOLD_MS;
    f.checkAt = f.brakeUntil + MOTOR_SETTLE_CHECK_MS;
    return;
  }
  if (arrived && now - f.arrivedAt > MOTOR_TIMEOUT_MS) {
    stopMove(f, n);
    Serial.printf("fader %d: gave up, %.3f from target\n", n + 1, err);
    return;
  }
  // Far behind the glide for a while: a hand is holding it (or it's blocked).
  if (fabsf(lag) > MOTOR_HELD_LAG) {
    if (!f.lagSince) f.lagSince = now;
    if (now - f.lagSince > MOTOR_HELD_MS) {
      stopMove(f, n);
      f.target = -1;  // so the main board's next target moves it again
      Serial.printf("fader %d: held, letting go\n", n + 1);
      return;
    }
  } else {
    f.lagSince = 0;
  }

  // Power: what the glide's speed needs (friction plus speed), plus a
  // correction for lagging behind it, and damping if it runs faster than the
  // glide. Friction is only added in the glide's direction, and in full only
  // at speed: flipping it with every tiny error made the motor chatter, and
  // in full on a short, slow move it jumped a loose fader past the target.
  // Stuck behind the glide: push harder, gradually, until it breaks free.
  float u = MOTOR_KP * lag + MOTOR_KD * (f.spVel - f.vel);
  if (fabsf(f.spVel) > 0.01f)
    u += copysignf(MOTOR_D0 * min(1.0f, fabsf(f.spVel) / MOTOR_FULL_FRICTION_VEL) + fabsf(f.spVel) / MOTOR_K, f.spVel);
  else if (fabsf(lag) <= MOTOR_TOL)
    u = 0;
  if (fabsf(f.smooth - f.progressPos) > 0.004f || fabsf(lag) <= MOTOR_TOL) {
    f.progressPos = f.smooth;
    f.progressAt = now;
  }
  uint32_t stuck = now - f.progressAt;
  if (stuck > MOTOR_STALL_MS) u += copysignf((float)(stuck - MOTOR_STALL_MS) / MOTOR_KICK_MS, lag);
  drive(n, constrain(u, -1.0f, 1.0f));
}

float averageRaw(int n) {
  long sum = 0;
  for (int k = 0; k < 16; k++) sum += analogRead(WIPER[n]);
  return sum / 16.0f;
}

// Drive every fader one way (dir -1 down, 1 up) until it stops against its
// end, and record the reading there.
void calibrateEnds(float dir, float *end) {
  const float POWER = 0.7f;
  const uint32_t RAMP_MS = 80, MIN_MS = 150, STILL_MS = 40, STILL_COUNTS = 12;
  float last[MAX_FADERS];
  uint32_t stillSince[MAX_FADERS];
  bool done[MAX_FADERS];
  float first[MAX_FADERS];
  uint32_t movedAt[MAX_FADERS];  // DEBUG: when each fader got going
  for (int n = 0; n < FADERS; n++) {
    last[n] = first[n] = analogRead(WIPER[n]);
    stillSince[n] = 0;
    done[n] = false;
    movedAt[n] = 0;
  }
  uint32_t t0 = millis();
  for (int left = FADERS; left > 0;) {
    uint32_t t = millis() - t0;
    float power = POWER * min(1.0f, (float)t / RAMP_MS);
    for (int n = 0; n < FADERS; n++) {
      if (done[n]) continue;
      float raw = analogRead(WIPER[n]);
      if (!movedAt[n] && fabsf(raw - first[n]) > 60) movedAt[n] = t;
      if (fabsf(raw - last[n]) > STILL_COUNTS) { last[n] = raw; stillSince[n] = t; }
      // Stopped for a while (or out of time): it's at its end.
      if ((t > MIN_MS && t - stillSince[n] > STILL_MS) || t > CAL_DRIVE_MS + 300) {
        coast(n);
        end[n] = averageRaw(n);
        done[n] = true;
        left--;
      } else {
        drive(n, dir * power);
      }
    }
    delay(1);
  }
  Serial.printf("cal %s: started from", dir > 0 ? "up  " : "down");
  for (int n = 0; n < FADERS; n++) Serial.printf(" %4.0f", first[n]);
  Serial.printf("\ncal %s: moved at ms ", dir > 0 ? "up  " : "down");
  for (int n = 0; n < FADERS; n++) Serial.printf(" %4lu", movedAt[n]);
  Serial.println();
  delay(60);  // let them settle before the other way
}

// Drive every fader into both end stops and record them. This also finds
// which way each motor is wired, so motor leads can go either way round.
void calibrate() {
  Serial.println("Calibrating faders");
  float lo[MAX_FADERS], hi[MAX_FADERS];
  // All together: every fader dives into its bottom stop, then all spring to
  // the top. Eight motors pushing into their stops at full power is more
  // current than the supply gives (the drivers cut out), so the power ramps
  // up softly, stays below full, and each fader lets go as soon as it has
  // stopped against its end.
  for (int n = 0; n < FADERS; n++) faders[n].flip = false;
  calibrateEnds(-1.0f, lo);
  calibrateEnds(1.0f, hi);
  for (int n = 0; n < FADERS; n++) {
    Fader &f = faders[n];
    float a = min(lo[n], hi[n]), b = max(lo[n], hi[n]);
    f.enabled = b - a >= CAL_MIN_SPAN;
    if (!f.enabled) {  // never drive a fader we can't read
      coast(n);
      f.moving = false;
      Serial.printf("fader %d: calibration failed (%.0f-%.0f), disabled - check its wiring\n", n + 1, a, b);
      continue;
    }
    float gap = (b - a) * CAL_END_GAP;
    f.minRaw = a + gap;
    f.maxRaw = b - gap;
    f.smooth = -1;
    // "Down" should lower the position; if it raised it, the motor is the other way round.
    f.flip = (lo[n] > hi[n]) != POT_REVERSE[n];
    Serial.printf("fader %d: %.0f-%.0f%s\n", n + 1, f.minRaw, f.maxRaw, f.flip ? " (motor reversed)" : "");
    // Targets that arrived while calibrating were never moved to; forget
    // them so the main board's next re-send moves the fader.
    f.target = -1;
  }
}

// ---------- touch ----------

// Touch by charge time: discharge the pad, release it to the internal
// pull-up, and count CPU cycles until it reads high. A finger on the fader
// lever adds capacitance and slows the charge. (The ESP32's touch peripheral
// wouldn't start scanning with several pads on this board.) The pull-up stays
// on; only the output driver is switched, straight in the GPIO registers, and
// interrupts are held off while timing so other work can't stretch a reading.
// The median of 15 drops the odd one that motor noise cuts short or stretches.
portMUX_TYPE touchMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t chargeTime(uint8_t pin) {
  const uint32_t bit = 1UL << pin;
  const uint32_t LIMIT = 50000;  // ~0.2 ms: a pad that never charges reads this
  uint32_t s[15];
  pinMode(pin, INPUT_PULLUP);
  for (int k = 0; k < 15; k++) {
    REG_WRITE(GPIO_OUT_W1TC_REG, bit);
    REG_WRITE(GPIO_ENABLE_W1TS_REG, bit);  // drive low
    delayMicroseconds(3);
    portENTER_CRITICAL(&touchMux);
    uint32_t start = ESP.getCycleCount();
    REG_WRITE(GPIO_ENABLE_W1TC_REG, bit);  // let the pull-up charge it
    uint32_t t;
    do t = ESP.getCycleCount() - start;
    while (!(REG_READ(GPIO_IN_REG) & bit) && t < LIMIT);
    portEXIT_CRITICAL(&touchMux);
    s[k] = t;
  }
  pinMode(pin, INPUT);  // park floating: holding the line low stalls the motors
  for (int i = 1; i < 15; i++)
    for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { uint32_t x = s[j]; s[j] = s[j - 1]; s[j - 1] = x; }
  return s[7];
}

// A finger only raises the reading, so the baseline drops to any lower
// reading at once and follows upward drift slowly while untouched. A touch
// has to be well over the baseline for several readings in a row, so a
// noise spike can't let go of a fader; letting go takes a clear drop.
bool nativeTouched(Fader &f, int n) {
  float v = chargeTime(TOUCH_PIN[n]);
  f.touchLast = v;
  bool was = f.padTouched;
  if (f.padTouched) {
    f.padTouched = v > f.touchBase * (1.0f + TOUCH_OFF);
  } else if (v > f.touchBase * (1.0f + TOUCH_ON)) {
    f.padTouched = ++f.touchCount >= TOUCH_CONFIRM;
  } else {
    f.touchCount = 0;
    if (v < f.touchBase) f.touchBase = v;
    else if (v < f.touchBase * (1.0f + TOUCH_OFF)) f.touchBase += 0.02f * (v - f.touchBase);
  }
  if (f.padTouched) f.touchCount = 0;
  if (f.padTouched != was)  // DEBUG
    Serial.printf("fader %d %s (%.0f/%.0f)\n", n + 1, f.padTouched ? "touched" : "released", v, f.touchBase);
  return f.padTouched;
}

void calibrateNativeTouch() {
  for (int n = 0; n < FADERS; n++) {
    float sum = 0;
    for (int k = 0; k < 8; k++) sum += chargeTime(TOUCH_PIN[n]);
    faders[n].touchBase = sum / 8;
    // Cycles to charge: a pad that never charges (shorted) or charges
    // instantly (nothing on it) isn't usable, so rely on motion instead.
    faders[n].touchOk = faders[n].touchBase > 20 && faders[n].touchBase < 40000;
    Serial.printf("fader %d touch baseline %.0f%s\n", n + 1, faders[n].touchBase,
                  faders[n].touchOk ? "" : " (not usable, detecting hands by motion)");
  }
}

void updateTouch(uint32_t now) {
  uint16_t mask = haveMpr ? mpr.touched() : 0;
  static uint16_t lastMask = 0;  // DEBUG: log each pad touched or let go
  if (haveMpr && mask != lastMask) {
    for (int n = 0; n < FADERS; n++) {
      uint16_t b = 1 << MPR_ELECTRODE[n];
      if ((mask ^ lastMask) & b)
        Serial.printf("[%lu] fader %d pad %s (%d/%d)\n", now, n + 1, mask & b ? "touched" : "released",
                      mpr.filteredData(MPR_ELECTRODE[n]), mpr.baselineData(MPR_ELECTRODE[n]));
    }
    lastMask = mask;
  }
  for (int n = 0; n < FADERS; n++) {
    Fader &f = faders[n];
    if (!f.enabled) { f.touched = false; continue; }
    bool raw = false;
    if (NATIVE_TOUCH && f.touchOk) raw = nativeTouched(f, n);
    else if (haveMpr) raw = mask & (1 << MPR_ELECTRODE[n]);

    // Also a hand: the fader moving while its motor isn't driving it. This
    // covers a touch pad that isn't working (or isn't fitted).
    if (f.moving || f.brakeUntil || f.checkAt || f.smooth < 0) {
      f.restPos = f.smooth;  // the motor is moving it, not a hand
      f.idleSince = now;
    } else if (now - f.idleSince > MOTION_SETTLE_MS) {  // let it coast to rest first
      if (f.restPos < 0) f.restPos = f.smooth;
      if (fabsf(f.smooth - f.restPos) > NOTOUCH_MOVE) { f.restPos = f.smooth; f.movedAt = now; }
      if (f.movedAt && now - f.movedAt < NOTOUCH_STILL_MS) raw = true;
    } else {
      f.restPos = f.smooth;
    }
    if (raw) {
      f.releasedAt = 0;
      f.touched = true;  // the motor task lets go of the fader
    } else if (f.touched) {
      if (!f.releasedAt) f.releasedAt = now;
      if (now - f.releasedAt >= TOUCH_HOLD_MS) f.touched = false;
    }
  }
}

// ---------- displays ----------

// Each fader has a 0.96" SSD1306 OLED (128x64, I2C 0x3C) showing what it's
// assigned to: label and level along the top, the name as large as it fits,
// and MUTE / SOLO along the bottom. The main board sends the label, name,
// mute and solo ("N" lines); the level comes from the fader itself.
//
// All eight answer at 0x3C, so each sits on its own channel of a TCA9548A
// multiplexer (0x70) on the MPR121's bus: display n on channel n-1. One U8g2
// object and one screen buffer serve them all; selecting a channel points it
// at that display. The MPR121 sits upstream of the multiplexer, so touch
// reads work whichever channel is selected. Without a multiplexer, a single
// OLED wired straight to the bus shows fader 1 (the original test setup).
//
// Sending a whole screen takes ~25 ms at 400 kHz, which would hold up touch
// reads on the same bus, so a changed screen goes out one 8-pixel row per
// pass of loop() (~3 ms each). Displays that need redrawing take turns.
const uint8_t TCA_ADDR = 0x70, OLED_ADDR = 0x3C;
const uint32_t DISPLAY_MIN_MS = 40;   // redraw a display at most this often

struct FaderInfo {
  char label[8] = "";                 // e.g. "CH 1", "BUS 3"; empty = unassigned
  char name[16] = "";                 // from the mixer
  bool on = true;                     // false = muted
  bool solo = false;
};
FaderInfo info[MAX_FADERS];
uint32_t infoSeq[MAX_FADERS];         // bumped when a fader's info changes

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);
bool haveTca = false;
bool displayOn[MAX_FADERS];           // a display answered for this fader
struct DisplayState { uint32_t drawnSeq = ~0u, lastDraw = 0; int drawnLevel = -2; };
DisplayState dstate[MAX_FADERS];
int tcaChannel = -2;                  // channel currently selected (-1 none, -2 unknown)

// X-Air fader law: 0.0-1.0 maps to -inf..+10 dB in four linear segments.
float faderToDb(float f) {
  if (f >= 0.5f)    return f * 40.0f - 30.0f;
  if (f >= 0.25f)   return f * 80.0f - 50.0f;
  if (f >= 0.0625f) return f * 160.0f - 70.0f;
  return f * 480.0f - 90.0f;
}

// Point the I2C bus at fader n's display (-1: no channel).
void selectDisplay(int n) {
  if (!haveTca || n == tcaChannel) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(n < 0 ? 0 : 1 << n);
  Wire.endTransmission();
  tcaChannel = n;
}

bool oledAnswers() {
  Wire.beginTransmission(OLED_ADDR);
  return Wire.endTransmission() == 0;
}

void displayBegin() {
  disp.setBusClock(400000);
  Wire.beginTransmission(TCA_ADDR);
  haveTca = Wire.endTransmission() == 0;
  int found = 0;
  if (haveTca) {
    for (int n = 0; n < FADERS; n++) {
      selectDisplay(n);
      displayOn[n] = oledAnswers();
      if (displayOn[n]) {
        disp.begin();  // starts and clears the display on this channel
        disp.setContrast(255);
        found++;
      }
    }
    selectDisplay(-1);
    Serial.printf("Fader displays: TCA9548A found, %d of %d displays (", found, FADERS);
    for (int n = 0; n < FADERS; n++) Serial.printf("%s%c", n ? " " : "", displayOn[n] ? '1' + n : '-');
    Serial.println(")");
  } else if (oledAnswers()) {
    displayOn[0] = true;
    disp.begin();
    disp.setContrast(255);
    Serial.println("Fader displays: no multiplexer, one display (fader 1)");
  } else {
    Serial.println("Fader displays: none found");
  }
}

// Draw fader n's screen into the buffer:
//   CH 1          -12.5      label, and the level in the corner
//         Kick               name, as large as fits
//   [MUTE]        [SOLO]     filled when on, outlined when off
void drawBadge(int x, const char *text, bool lit) {
  const int w = 40, y = 52, h = 12;
  if (lit) {
    disp.drawRBox(x, y, w, h, 2);
    disp.setDrawColor(0);
  } else {
    disp.drawRFrame(x, y, w, h, 2);
  }
  disp.drawStr(x + (w - disp.getStrWidth(text)) / 2, y + 10, text);
  disp.setDrawColor(1);
}

void drawFaderScreen(int n, float level) {
  const FaderInfo &in = info[n];
  disp.clearBuffer();
  disp.setFont(u8g2_font_6x10_tr);
  if (!in.label[0]) {  // nothing on this fader in this bank
    disp.drawStr((128 - disp.getStrWidth("--")) / 2, 36, "--");
    return;
  }
  disp.drawStr(0, 9, in.label);
  char db[10];
  if (level < 0.0f) snprintf(db, sizeof(db), "--");
  else if (level <= 0.0f) snprintf(db, sizeof(db), "-inf");
  else snprintf(db, sizeof(db), "%+.1f", faderToDb(level));
  disp.drawStr(128 - disp.getStrWidth(db), 9, db);

  drawBadge(0, "MUTE", !in.on);
  drawBadge(88, "SOLO", in.solo);

  // Name, as big as fits (the label if it has no name), between the top
  // line and the badges.
  const char *name = in.name[0] ? in.name : in.label;
  const uint8_t *fonts[] = {u8g2_font_helvB18_tr, u8g2_font_helvB14_tr, u8g2_font_helvB10_tr, u8g2_font_6x12_tr};
  for (const uint8_t *font : fonts) {
    disp.setFont(font);
    if (disp.getStrWidth(name) <= 128 || font == u8g2_font_6x12_tr) break;
  }
  const int top = 12, bottom = 50;
  disp.drawStr(max(0, (128 - disp.getStrWidth(name)) / 2), (top + bottom + disp.getAscent()) / 2, name);
}

void displayService(uint32_t now) {
  static int sending = -1, sendRow = 0;  // display being sent, its next row
  static int next = 0;                   // where the next look for work starts
  if (sending >= 0) {
    selectDisplay(sending);
    disp.updateDisplayArea(0, sendRow, 16, 1);
    if (++sendRow >= 8) sending = -1;
    return;
  }
  for (int k = 0; k < FADERS; k++) {
    int n = (next + k) % FADERS;
    if (!displayOn[n]) continue;
    const Fader &f = faders[n];
    // Show where the fader is while a hand moves it, otherwise its level.
    float level = f.touched || f.target < 0 ? f.smooth : f.target;
    int shown = level < 0 ? -1 : (int)lroundf(level * 1023);  // the mixer's 1023 steps
    DisplayState &d = dstate[n];
    if ((shown != d.drawnLevel || infoSeq[n] != d.drawnSeq) && now - d.lastDraw >= DISPLAY_MIN_MS) {
      d.lastDraw = now;
      d.drawnLevel = shown;
      d.drawnSeq = infoSeq[n];
      drawFaderScreen(n, level);
      sending = n;
      sendRow = 0;
      next = n + 1;  // then give the others a turn
      return;
    }
  }
}

// "N <n> <on> <solo> <label>|<name>" from the main board.
void handleInfo(char *line) {
  int n, on, solo, used = 0;
  if (sscanf(line + 1, "%d %d %d %n", &n, &on, &solo, &used) < 3 || !used || n < 0 || n >= MAX_FADERS) return;
  char *label = line + 1 + used, *bar = strchr(label, '|');
  const char *name = bar ? bar + 1 : "";
  if (bar) *bar = 0;
  FaderInfo &in = info[n];
  in.on = on != 0;
  in.solo = solo != 0;
  strlcpy(in.label, label, sizeof(in.label));
  strlcpy(in.name, name, sizeof(in.name));
  infoSeq[n]++;
}

// ---------- link ----------

void sendHello() { Serial1.printf("H %d %d\n", FADERS, NATIVE_TOUCH || haveMpr ? 1 : 0); }

// Requests for the motor task (which owns the motors and fader state).
volatile float requestTarget[MAX_FADERS];
volatile uint32_t requestSeq[MAX_FADERS];
volatile bool calRequest = false;
volatile bool introRequest = false;  // play the power-up show after calibrating

volatile bool usbControl = false;  // DEBUG: USB "G"/"S" commands own the targets until "L"

void handleLine(char *line, bool fromUsb = false) {
  if (line[0] == 'T' && (fromUsb || !usbControl)) {
    int n, pos;
    if (sscanf(line + 1, "%d %d", &n, &pos) == 2 && n >= 0 && n < FADERS) {
      requestTarget[n] = pos < 0 ? -1.0f : constrain(pos, 0, 1000) / 1000.0f;
      requestSeq[n] = requestSeq[n] + 1;
    }
  } else if (line[0] == 'N') {
    handleInfo(line);
  } else if (line[0] == 'C') {
    calRequest = true;
  } else if (line[0] == '?') {
    sendHello();
  }
}

uint32_t linkBytes = 0, linkCommands = 0;

void readLink() {
  static char buf[64];
  static int len = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    linkBytes++;
    if (c == '\n') linkCommands++;
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void report() {
  uint16_t mask = 0;
  for (int n = 0; n < FADERS; n++)
    if (faders[n].touched) mask |= 1 << n;
  char out[80];
  int len = snprintf(out, sizeof(out), "S %u", mask);
  for (int n = 0; n < MAX_FADERS; n++) {
    int p = n < FADERS && faders[n].enabled && faders[n].smooth >= 0 ? (int)lroundf(faders[n].smooth * 1000) : -1;
    len += snprintf(out + len, sizeof(out) - len, " %d", p);
  }
  Serial1.println(out);
}

// ---------- main ----------

void setup() {
  // Motors off first thing: until then some driver inputs float, and G39
  // (fader 5's IN2) is pulled high during boot, which runs fader 5 up. A
  // 10k pull-down from G39 to GND stops it before this code even starts.
  for (int n = 0; n < FADERS; n++) {
    initMotor(n);
    coast(n);
  }
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  analogReadResolution(12);
  if (NATIVE_TOUCH) {
    Serial.printf("Fader board: %d faders, touch on G5-G8\n", FADERS);
    calibrateNativeTouch();  // before the motors run, with no hands on the faders
  } else {
    Wire.begin(I2C_SDA, I2C_SCL);
    // Auto-setup on: the chip tunes each input's charge to its wire and fader.
    // (With it "off" the library still leaves auto re-tuning on, with no
    // targets, which fails and leaves every input reading near zero.)
    haveMpr = mpr.begin(0x5A, &Wire, 12, 6, true);
    displayBegin();
    Serial.printf("Fader board: %d faders, MPR121 %s\n", FADERS,
                  haveMpr ? "found" : "not found (detecting hands by motion)");
  }
  calRequest = true;  // calibrated by the motor task: from setup() the motors didn't move
  introRequest = true;
  sendHello();
  xTaskCreatePinnedToCore(motorTask, "motors", 4096, nullptr, 5, nullptr, 0);
}

// Motor control runs in its own task on the other core, every millisecond,
// so slow touch reads and link traffic in loop() can't make it overshoot.
// DEBUG motor test, typed over USB serial: "M <n>" pulses fader n's motor
// one way then the other and reports the raw wiper readings ("M 0" = all).
volatile int testRequest = -1;
volatile bool motorsOff = false;  // DEBUG: "X" over USB toggles all motors off

void testMotor(int n) {
  faders[n].moving = false;
  float a = averageRaw(n);
  drive(n, 1.0f);
  delay(250);
  float b = averageRaw(n);
  coast(n);
  delay(100);
  drive(n, -1.0f);
  delay(250);
  float c = averageRaw(n);
  coast(n);
  Serial.printf("test fader %d: start %.0f, after forward %.0f, after reverse %.0f -> %s\n", n + 1, a, b, c,
                fabsf(b - a) > 300 || fabsf(c - b) > 300 ? "MOVES" : "no movement");
}

// Power-up show, after calibration: a wave rolls across the faders, calms
// until they all meet in a line at the middle, then they drop like dominoes
// to the bottom. Positions go through setTarget() 50 times a second, the same
// smooth glide as a stream from the app, so it flows; a hand on a fader
// stops that fader as usual. Levels from the main board that arrive during
// it are picked up straight after.
float smoothstep(float a, float b, float x) {
  float t = constrain((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

void playIntro() {
  const float WAVE_S = 1.2f;    // one wave period
  const float FLOW_END = 1.5f;  // wave rolls until here...
  const float CALM_END = 2.5f;  // ...calms into a line by here...
  const float DROP_GAP = 0.07f; // ...then drops, one fader after another
  const float END = CALM_END + DROP_GAP * FADERS + 0.6f;
  float startPos[MAX_FADERS];
  for (int n = 0; n < FADERS; n++) {
    startPos[n] = readPosition(n);
    faders[n].smooth = startPos[n];
  }
  uint32_t t0 = millis(), lastSend = 0, lastUs = micros();
  for (;;) {
    uint32_t now = millis();
    float t = (now - t0) / 1000.0f;
    if (t > END) break;
    if (now - lastSend >= 20) {
      lastSend = now;
      for (int n = 0; n < FADERS; n++) {
        float phase = t / WAVE_S - (float)n / FADERS;
        float amp = 0.38f * smoothstep(0.0f, 0.8f, t) * (1.0f - smoothstep(FLOW_END, CALM_END, t));
        float wave = 0.5f + amp * sinf(2.0f * PI * phase);
        // Ease in from where calibration left each fader, staggered left to right.
        float pos = startPos[n] + (wave - startPos[n]) * smoothstep(0.05f * n, 0.05f * n + 0.6f, t);
        float dropAt = CALM_END + DROP_GAP * n;
        if (t >= dropAt) pos = 0.5f * (1.0f - smoothstep(dropAt, dropAt + 0.35f, t));
        setTarget(n, pos, now);
      }
    }
    uint32_t us = micros();
    float dt = min((us - lastUs) / 1e6f, 0.01f);
    lastUs = us;
    for (int n = 0; n < FADERS; n++) updateFader(n, now, dt);
    vTaskDelay(1);
  }
}

// DEBUG "V <n>": speed at each motor power, both ways, from end stop to end stop.
volatile int sweepRequest = -1;
volatile int probeFader = -1, probeDuty = 0;  // DEBUG "P <n> <power -1000..1000>": hold that power 300 ms, recorded
void sweepMotor(int n) {
  Fader &f = faders[n];
  f.moving = false;
  for (int dir = 1; dir >= -1; dir -= 2) {
    for (int pct = 10; pct <= 60; pct += 5) {
      drive(n, -dir * 1.0f);  // back to the start end
      delay(350);
      coast(n);
      delay(80);
      float start = readPosition(n);
      uint32_t t0 = millis(), tA = 0, tB = 0;
      drive(n, dir * pct / 100.0f);
      while (millis() - t0 < 1500) {
        float p = readPosition(n);
        float q = dir > 0 ? p : 1.0f - p;
        if (!tA && q >= 0.3f) tA = millis();
        if (!tB && q >= 0.7f) { tB = millis(); break; }
        delay(1);
      }
      coast(n);
      float end = readPosition(n);
      Serial.printf("sweep %d %s %3d%%: %s", n + 1, dir > 0 ? "up  " : "down", pct,
                    tA && tB ? "" : "slow ");
      if (tA && tB) Serial.printf("%.2f travel/s\n", 0.4f / ((tB - tA) / 1000.0f));
      else Serial.printf("moved %.3f in 1.5 s\n", fabsf(end - start));
      delay(50);
    }
  }
}

// DEBUG "G <n> <pos 0-1000>": a target as if from the main board, recorded
// every 2 ms; "R" prints the recording (time, setpoint, position, power).
const int REC_LEN = 1500;
struct RecSample { uint16_t t; int16_t sp, pos, duty; };
RecSample rec[REC_LEN];
volatile int recFader = -1, recCount = 0;
uint32_t recStart = 0;

void startRecording(int n) {
  recCount = 0;
  recStart = millis();
  recFader = n;
}

void record(uint32_t now) {
  static uint32_t last = 0;
  if (recFader < 0 || recCount >= REC_LEN || now - last < 2) return;
  last = now;
  Fader &f = faders[recFader];
  rec[recCount] = {(uint16_t)(now - recStart), (int16_t)lroundf((f.sp >= 0 ? f.sp : f.target) * 1000), (int16_t)lroundf(f.smooth * 1000),
                   (int16_t)lroundf(lastDuty[recFader] * 1000)};
  recCount = recCount + 1;
}

void printRecording() {
  Serial.printf("rec fader %d, %d samples\n", recFader + 1, recCount);
  for (int i = 0; i < recCount; i++) Serial.printf("r %u %d %d %d\n", rec[i].t, rec[i].sp, rec[i].pos, rec[i].duty);
  Serial.println("rec end");
}

struct TargetStream { int fader; int from, to; uint32_t ms, stepMs, startAt, lastAt; bool uneven; uint32_t gap; };
TargetStream testStream = {-1, 0, 0, 0, 0, 0, 0};

void motorTask(void *) {
  uint32_t seen[MAX_FADERS] = {0};
  for (;;) {
    uint32_t now = millis();
    static uint32_t lastUs = micros();
    uint32_t us = micros();
    float dt = min((us - lastUs) / 1e6f, 0.01f);
    lastUs = us;
    if (calRequest) {
      calibrate();
      calRequest = false;
      if (introRequest) {
        playIntro();
        introRequest = false;
      }
    }
    if (probeFader >= 0) {
      faders[probeFader].moving = false;
      startRecording(probeFader);
      uint32_t t0 = millis();
      while (millis() - t0 < 600) {
        uint32_t t = millis() - t0;
        if (t >= 150 && t < 450) drive(probeFader, probeDuty / 1000.0f); else coast(probeFader);
        readPosition(probeFader);
        Fader &pf = faders[probeFader];
        pf.smooth += 0.3f * (readPosition(probeFader) - pf.smooth);
        record(millis());
        delay(1);
      }
      coast(probeFader);
      probeFader = -1;
    }
    if (sweepRequest >= 0) {
      sweepMotor(sweepRequest);
      sweepRequest = -1;
    }
    if (testRequest >= 0) {
      for (int n = 0; n < FADERS; n++)
        if (testRequest == 0 || testRequest == n + 1) testMotor(n);
      testRequest = -1;
    }
    for (int n = 0; n < FADERS; n++) {
      if (requestSeq[n] != seen[n]) {  // new target from the main board
        seen[n] = requestSeq[n];
        float t = requestTarget[n];
        Fader &f = faders[n];
        // The main board re-sends targets every couple of seconds in case a
        // line was lost. Only a changed target moves the fader; re-chasing the
        // same one made a fader that settles slightly off click every resend.
        if (!motorsOff && t != f.target) setTarget(n, t, now);
      }
      if (motorsOff && faders[n].moving) { faders[n].moving = false; coast(n); }
      updateFader(n, now, dt);
    }
    record(now);
    vTaskDelay(1);
  }
}

void loop() {
  static uint32_t lastTouch = 0, lastReport = 0;
  uint32_t now = millis();
  readLink();
  displayService(now);
  while (Serial.available()) {  // DEBUG: "M <n>" motor test over USB
    static char cmd[40];
    static int clen = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[clen] = 0;
      int n;
      if (clen && cmd[0] == 'M' && sscanf(cmd + 1, "%d", &n) == 1 && n >= 0 && n <= FADERS) testRequest = n;
      if (clen && cmd[0] == 'C') calRequest = true;
      if (clen && cmd[0] == 'V' && sscanf(cmd + 1, "%d", &n) == 1 && n >= 1 && n <= FADERS) sweepRequest = n - 1;
      int pd;
      if (clen && cmd[0] == 'P' && sscanf(cmd + 1, "%d %d", &n, &pd) == 2 && n >= 1 && n <= FADERS) { probeDuty = pd; probeFader = n - 1; }
      int pos, a, b, ms, step;
      if (clen && cmd[0] == 'G' && sscanf(cmd + 1, "%d %d", &n, &pos) == 2 && n >= 1 && n <= FADERS) {
        usbControl = true;
        startRecording(n - 1);
        char line[24];
        snprintf(line, sizeof(line), "T %d %d", n - 1, pos);
        handleLine(line, true);
      }
      // "S <n> <from> <to> <ms> <step ms> [u]": a stream of targets, like the app or
      // encoder; "u" spaces them unevenly (bunched or late), like the real link
      int un = 0;
      if (clen && cmd[0] == 'S' && sscanf(cmd + 1, "%d %d %d %d %d %d", &n, &a, &b, &ms, &step, &un) >= 5 &&
          n >= 1 && n <= FADERS && ms > 0 && step > 0) {
        usbControl = true;
        startRecording(n - 1);
        testStream = {n - 1, a, b, (uint32_t)ms, (uint32_t)step, now, 0, un != 0, (uint32_t)step};
      }
      if (clen && cmd[0] == 'R') printRecording();
      int cdc, cdt;  // DEBUG "Q <charge uA 1-63> <time code 1-7>": MPR121 charge settings, then readings
      if (clen && cmd[0] == 'Q' && haveMpr && sscanf(cmd + 1, "%d %d", &cdc, &cdt) == 2) {
        mpr.writeRegister(0x5E, 0);  // stop to change settings
        mpr.writeRegister(0x5C, 0x00 | (cdc & 0x3F));
        mpr.writeRegister(0x5D, (cdt & 7) << 5);
        mpr.writeRegister(0x5E, 0x8C);
        delay(150);
        Serial.printf("Q %duA t%d:", cdc, cdt);
        for (int e = 0; e < 12; e++) Serial.printf(" %d", mpr.filteredData(e));
        Serial.println();
      }
      if (clen && cmd[0] == 'L') { usbControl = false; Serial.println("main board in control"); }
      if (clen && cmd[0] == 'X') {
        motorsOff = !motorsOff;
        Serial.printf("motors %s\n", motorsOff ? "OFF" : "on");
      }
      clen = 0;
    } else if (clen < 39) {
      cmd[clen++] = c;
    }
  }
  if (testStream.fader >= 0 && now - testStream.lastAt >= testStream.gap) {  // DEBUG target stream
    testStream.lastAt = now;
    testStream.gap = !testStream.uneven ? testStream.stepMs
                     : random(100) < 30 ? 2 : random(testStream.stepMs / 2, testStream.stepMs * 4);
    float k = min(1.0f, (float)(now - testStream.startAt) / testStream.ms);
    char line[24];
    snprintf(line, sizeof(line), "T %d %d", testStream.fader, (int)lroundf(testStream.from + k * (testStream.to - testStream.from)));
    handleLine(line, true);
    if (k >= 1.0f) testStream.fader = -1;
  }
  if (now - lastTouch >= TOUCH_READ_MS) { lastTouch = now; updateTouch(now); }
  if (now - lastReport >= REPORT_MS) { lastReport = now; report(); }

  static uint32_t lastDebug = 0;  // DEBUG: link and touch status every 2 s
  if (now - lastDebug >= 2000) {
    lastDebug = now;
    Serial.printf("[%lu] link rx %lu bytes, %lu commands | pos", now, linkBytes, linkCommands);
    for (int n = 0; n < FADERS; n++)
      Serial.printf(" %.3f>%.3f%s%s(raw %d, cal %.0f-%.0f)", faders[n].smooth, faders[n].target,
                    faders[n].moving ? "m" : "", faders[n].flip ? "R" : "", (int)faders[n].lastRaw,
                    faders[n].minRaw, faders[n].maxRaw);
    Serial.print(" | touch");
    for (int n = 0; n < FADERS; n++) {
      if (haveMpr)  // MPR121: filtered reading / its baseline (a touch drops it 12+ below)
        Serial.printf(" %s%d/%d", faders[n].touched ? "*" : "", mpr.filteredData(MPR_ELECTRODE[n]),
                      mpr.baselineData(MPR_ELECTRODE[n]));
      else
        Serial.printf(" %s%.0f/%.0f", faders[n].touched ? "*" : "", faders[n].touchLast, faders[n].touchBase);
    }
    if (haveMpr) {
      Serial.printf(" mask %03x | all", mpr.touched());
      for (int e = 0; e < 12; e++) Serial.printf(" %d", mpr.filteredData(e));
      Serial.printf(" | ECR %02x OOR %02x%02x CFG %02x %02x", mpr.readRegister8(0x5E), mpr.readRegister8(0x03),
                    mpr.readRegister8(0x02), mpr.readRegister8(0x5C), mpr.readRegister8(0x5D));
    }
    Serial.println();
  }
}
