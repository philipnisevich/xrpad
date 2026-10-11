// XR18 fader board — a second ESP32-S3 that runs up to 8 motorized faders
// for the xr_console main board. It has no Wi-Fi: it only moves faders to
// the positions the main board asks for, and reports where they are and
// which ones a hand is on.
//
// Each fader runs the same motor control as the console's single fader:
// full power until it breaks away, gentle near the target, a power kick if
// it stalls, and -inf/+10 dB drive into the end stops. A hand on a fader
// stops its motor at once.
//
// Wiring (logic is 3V3; motor drivers on their own 5-7 V supply, grounds shared):
//   Fader n wiper (n = 1-8)    -> G1..G8   (other ends 3V3 / GND)
//   Fader n motor driver IN1   -> G9..G16  (PWM)
//   Fader n motor driver IN2   -> G17, G18, G21, G38, G39, G40, G41, G42
//   Fader n motor outputs      -> TC1508A motor A/B (two faders per board)
//   Fader n touch track        -> MPR121 ELE0..ELE7
//   MPR121                     SDA -> G47, SCL -> G48, VCC -> 3V3, GND
//   Link to main board         TX G48 -> main G14, RX G47 <- main G13, GND
//                              (G43/G44 aren't on the LCKFB board's headers.
//                              With the MPR121 on G47/G48 the link moves to
//                              TX G45 / RX G46; then unplug the link to flash
//                              this board, as G46 held high blocks download mode.)
//
// Temporary 4-fader setup without an MPR121 (NATIVE_TOUCH): faders 1-4 only,
// with their touch tracks on the ESP32's own touch pins G5..G8 (the wiper
// pins of faders 5-8, which are unused then). Keep these wires short.
//
// Protocol (921600 baud, text lines):
//   main -> board  "T <n> <pos>"  move fader n (0-7) to pos 0-1000; pos < 0 = no target
//                  "C"            recalibrate all faders
//                  "?"            ask for the hello line
//   board -> main  "H <faders> <touch sensor>"     hello (at boot and on "?")
//                  "S <touchmask> <p0> ... <p7>"   every 20 ms, positions 0-1000

#include <Wire.h>
#include <Adafruit_MPR121.h>

const int MAX_FADERS = 8;
const int FADERS = 4;               // how many are fitted (1 for testing with a single fader)
const bool NATIVE_TOUCH = true;     // true: touch on G5-G8 (4 faders max); false: MPR121
const uint8_t TOUCH_PIN[4] = {5, 6, 7, 8};
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
const float    MOTOR_MIN_DUTY = 0.65f;
const float    MOTOR_KP = 5.0f;
const float    MOTOR_TOL = 0.008f;
const float    MOTOR_PROGRESS = 0.01f;   // getting this much closer counts as progress
const uint32_t MOTOR_BRAKE_MS = 30;
const uint32_t MOTOR_SETTLE_CHECK_MS = 60; // after braking, wait this long then check the position
const uint8_t  MOTOR_CORRECTIONS = 3;     // gentle nudges back if it coasted past
const uint32_t MOTOR_STALL_MS = 30;
const uint32_t MOTOR_KICK_MS = 50;
const uint32_t MOTOR_BREAKAWAY_MS = 60;
const uint32_t MOTOR_END_MS = 80;
const uint32_t MOTOR_TIMEOUT_MS = 1500;
const uint32_t MOTOR_STAGGER_MS = 25;    // start delay between faders moving together
const uint32_t FOLLOW_MS = 300;          // targets closer together than this = a stream to follow

const uint32_t CAL_DRIVE_MS = 500;
const float    CAL_END_GAP = 0.015f;    // -inf/+10 dB sit this far in from the end stops
const int      CAL_MIN_SPAN = 1800;     // raw ADC counts; less = calibration failed

const uint32_t TOUCH_READ_MS = 15;
const uint32_t TOUCH_HOLD_MS = 150;     // stay "touched" briefly after letting go
const float    TOUCH_ON = 0.15f;        // native touch: this far above baseline = touched
const float    TOUCH_OFF = 0.08f;       // ...and below this = released
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
  bool moving = false, breakaway = false;
  volatile bool touched = false;     // set by loop(), acted on by the motor task
  float startErr = 0, bestErr = 0;
  uint32_t startAt = 0, progressAt = 0, brakeUntil = 0, releasedAt = 0;
  float restPos = -1;                // no-sensor hand detection
  uint32_t movedAt = 0;
  float touchBase = 0;               // native touch: untouched reading
  bool touchOk = false;              // native touch reading looks sane (else motion only)
  uint32_t idleSince = 0;            // when the motor last stopped
  float lastRaw = 0;                 // last wiper reading (for the debug line)
  uint32_t startAfter = 0;           // staggered start of a move
  uint32_t checkAt = 0;              // when to check where a move really stopped
  uint8_t corrections = 0;           // settle-back nudges used by this move
  bool following = false;            // tracking a stream of targets (see followTarget)
  uint32_t lastTargetAt = 0;
  bool enabled = false;              // calibrated; a fader that failed is never driven
  bool flip = false;                 // motor wired the other way round (found at calibration)
};

Fader faders[MAX_FADERS];
Adafruit_MPR121 mpr;
bool haveMpr = false;

// ---------- motor ----------

// IN1 carries PWM, IN2 the direction. Forward: IN2 low, IN1 = duty (drive /
// coast). Reverse: IN2 high, IN1 = 1 - duty (drive / brake). The control
// loop is closed, so the slightly different decay each way doesn't matter.
void drive(int n, float speed) {
  if (MOTOR_REVERSE[n] != faders[n].flip) speed = -speed;
  if (speed >= 0.0f) {
    digitalWrite(IN2[n], LOW);
    ledcWrite(IN1[n], (uint32_t)(speed * PWM_MAX));
  } else {
    digitalWrite(IN2[n], HIGH);
    ledcWrite(IN1[n], (uint32_t)((1.0f + speed) * PWM_MAX));
  }
}

void coast(int n) { digitalWrite(IN2[n], LOW); ledcWrite(IN1[n], 0); }
void brake(int n) { digitalWrite(IN2[n], HIGH); ledcWrite(IN1[n], PWM_MAX); }

// Median of three reads, so a single dropout from a loose wiper is ignored.
float readPosition(int n) {
  Fader &f = faders[n];
  int a = analogRead(WIPER[n]), b = analogRead(WIPER[n]), c = analogRead(WIPER[n]);
  float raw = max(min(a, b), min(max(a, b), c));
  f.lastRaw = raw;
  float pos = constrain((raw - f.minRaw) / (f.maxRaw - f.minRaw), 0.0f, 1.0f);
  return POT_REVERSE[n] ? 1.0f - pos : pos;
}

void startMove(int n, float target) {
  Fader &f = faders[n];
  // Only take the target once the move can start: a target that couldn't be
  // acted on (fader not read yet, a hand on it) must not look "already done"
  // when the main board re-sends it.
  if (!f.enabled || target < 0 || f.touched || f.smooth < 0) {
    f.moving = false;
    f.target = -1;
    return;
  }
  f.target = target;
  f.moving = true;
  f.breakaway = true;
  // Stagger starts so faders moving together don't all draw their
  // full-power start-up current at the same instant.
  f.startAfter = millis() + n * MOTOR_STAGGER_MS;
  f.startAt = f.progressAt = f.startAfter;
  f.corrections = 0;
  f.checkAt = 0;
  f.startErr = f.bestErr = fabsf(target - f.smooth);
}

// Steer towards a target that's part of a stream: keep driving (no
// full-power restart, no braking), just aim at the newest value.
void followTarget(int n, float target, uint32_t now) {
  Fader &f = faders[n];
  f.target = target;
  f.following = true;
  f.checkAt = 0;
  f.brakeUntil = 0;
  if (!f.moving) {
    if (fabsf(target - f.smooth) <= MOTOR_TOL) return;
    f.moving = true;
    f.breakaway = false;
    f.startAfter = now;
  }
  f.startAt = f.progressAt = now;
  f.startErr = f.bestErr = fabsf(target - f.smooth);
}

void updateFader(int n, uint32_t now) {
  Fader &f = faders[n];
  float pos = readPosition(n);
  f.smooth = f.smooth < 0 ? pos : f.smooth + 0.3f * (pos - f.smooth);

  if (f.brakeUntil && now >= f.brakeUntil) { coast(n); f.brakeUntil = 0; }

  // The stream has stopped: finish on the last target precisely.
  if (f.following && now - f.lastTargetAt > FOLLOW_MS) {
    f.following = false;
    if (!f.moving && !f.touched && f.target >= 0 && fabsf(f.target - f.smooth) > MOTOR_TOL * 2) {
      f.moving = true;
      f.breakaway = false;
      f.corrections = 0;
      f.startAfter = f.startAt = f.progressAt = now;
      f.startErr = f.bestErr = fabsf(f.target - f.smooth);
    }
  }

  // A fast move can coast past the target after braking: once it has
  // stopped, nudge it back gently (no full-power start), a few times at most.
  if (!f.moving && f.checkAt && now >= f.checkAt) {
    f.checkAt = 0;
    float e = f.target - f.smooth;
    if (f.target >= 0 && !f.touched && fabsf(e) > MOTOR_TOL * 2 && f.corrections < MOTOR_CORRECTIONS) {
      f.corrections++;
      f.moving = true;
      f.breakaway = false;
      f.startAfter = f.startAt = f.progressAt = now;
      f.startErr = f.bestErr = fabsf(e);
    }
  }
  if (!f.moving || (int32_t)(now - f.startAfter) < 0) return;  // not started yet

  float err = f.target - f.smooth;
  bool endTarget = f.target <= 0.0f || f.target >= 1.0f;
  // Progress = actually getting closer; position noise mustn't count.
  if (fabsf(err) < f.bestErr - MOTOR_PROGRESS) {
    f.bestErr = fabsf(err);
    f.progressAt = now;
    f.breakaway = false;
  }
  if (now - f.startAt > MOTOR_BREAKAWAY_MS) f.breakaway = false;
  uint32_t stuck = now - f.progressAt;

  if (f.touched) {
    coast(n);
    f.moving = false;
  } else if (endTarget && stuck > MOTOR_END_MS) {  // pushed into the end stop
    coast(n);
    f.moving = false;
  } else if (!endTarget && fabsf(err) <= MOTOR_TOL) {
    f.moving = false;
    if (f.following) {
      coast(n);  // the next target is on its way: no braking jolt
    } else {
      brake(n);
      f.brakeUntil = now + MOTOR_BRAKE_MS;
      f.checkAt = now + MOTOR_BRAKE_MS + MOTOR_SETTLE_CHECK_MS;  // see where it actually stopped
    }
  } else if (!f.following && now - f.startAt > 60 && fabsf(err) > f.startErr + 0.03f) {
    // Going the wrong way. Calibration finds each motor's direction, so this
    // is almost always a hand pulling the fader: let go of it. (Reversing
    // the motor here once made a correctly-wired fader permanently backwards.)
    coast(n);
    f.moving = false;
    Serial.printf("fader %d: pulled away from its target, letting go\n", n + 1);
  } else if (now - f.startAt > MOTOR_TIMEOUT_MS) {
    coast(n);
    f.moving = false;
    Serial.printf("fader %d: gave up, %.3f from target\n", n + 1, err);
  } else {
    // Full power to break away, then gentle near the target, ramping back
    // up if it stalls.
    float kick = stuck > MOTOR_STALL_MS ? (float)(stuck - MOTOR_STALL_MS) / MOTOR_KICK_MS : 0.0f;
    float power = constrain(fabsf(err) * MOTOR_KP, MOTOR_MIN_DUTY, 1.0f);
    power = endTarget || f.breakaway ? 1.0f : min(power + kick, 1.0f);
    drive(n, err > 0.0f ? power : -power);
  }
}

float averageRaw(int n) {
  long sum = 0;
  for (int k = 0; k < 16; k++) sum += analogRead(WIPER[n]);
  return sum / 16.0f;
}

// Drive every fader into both end stops and record them. This also finds
// which way each motor is wired, so motor leads can go either way round.
void calibrate() {
  Serial.println("Calibrating faders");
  float lo[MAX_FADERS], hi[MAX_FADERS];
  // One fader at a time: every motor stalling at full power together is
  // more current than a modest supply can give, and the drivers cut out.
  for (int n = 0; n < FADERS; n++) {
    faders[n].flip = false;
    drive(n, -1.0f);
    delay(CAL_DRIVE_MS);
    lo[n] = averageRaw(n);
    coast(n);
    delay(50);
    drive(n, 1.0f);
    delay(CAL_DRIVE_MS);
    hi[n] = averageRaw(n);
    coast(n);
    delay(50);
  }
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

// Native touch: a finger only raises the reading, so the baseline drops to
// any lower reading at once and follows upward drift slowly while untouched.
bool nativeTouched(Fader &f, int n) {
  float v = touchRead(TOUCH_PIN[n]);
  bool over = v > f.touchBase * (1.0f + (f.touched ? TOUCH_OFF : TOUCH_ON));
  if (!over) {
    if (v < f.touchBase) f.touchBase = v;
    else if (v < f.touchBase * (1.0f + TOUCH_OFF)) f.touchBase += 0.02f * (v - f.touchBase);
  }
  return over;
}

void calibrateNativeTouch() {
  for (int n = 0; n < FADERS; n++) {
    for (int k = 0; k < 5; k++) touchRead(TOUCH_PIN[n]);  // first readings after boot run high
    float sum = 0;
    for (int k = 0; k < 8; k++) sum += touchRead(TOUCH_PIN[n]);
    faders[n].touchBase = sum / 8;
    // A good touch pad reads tens to hundreds of thousands; millions (or the
    // 4194303 ceiling) means the pin isn't working, so rely on motion instead.
    faders[n].touchOk = faders[n].touchBase > 1000 && faders[n].touchBase < 1000000;
    Serial.printf("fader %d touch baseline %.0f%s\n", n + 1, faders[n].touchBase,
                  faders[n].touchOk ? "" : " (not usable, detecting hands by motion)");
  }
}

void updateTouch(uint32_t now) {
  uint16_t mask = haveMpr ? mpr.touched() : 0;
  for (int n = 0; n < FADERS; n++) {
    Fader &f = faders[n];
    if (!f.enabled) { f.touched = false; continue; }
    bool raw = false;
    if (NATIVE_TOUCH && f.touchOk) raw = nativeTouched(f, n);
    else if (haveMpr) raw = mask & (1 << n);

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

// ---------- link ----------

void sendHello() { Serial1.printf("H %d %d\n", FADERS, NATIVE_TOUCH || haveMpr ? 1 : 0); }

// Requests for the motor task (which owns the motors and fader state).
volatile float requestTarget[MAX_FADERS];
volatile uint32_t requestSeq[MAX_FADERS];
volatile bool calRequest = false;

void handleLine(char *line) {
  if (line[0] == 'T') {
    int n, pos;
    if (sscanf(line + 1, "%d %d", &n, &pos) == 2 && n >= 0 && n < FADERS) {
      requestTarget[n] = pos < 0 ? -1.0f : constrain(pos, 0, 1000) / 1000.0f;
      requestSeq[n] = requestSeq[n] + 1;
    }
  } else if (line[0] == 'C') {
    calRequest = true;
  } else if (line[0] == '?') {
    sendHello();
  }
}

uint32_t linkBytes = 0, linkCommands = 0;

void readLink() {
  static char buf[48];
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
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  analogReadResolution(12);
  for (int n = 0; n < FADERS; n++) {
    pinMode(IN2[n], OUTPUT);
    ledcAttach(IN1[n], MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    coast(n);
  }
  if (NATIVE_TOUCH) {
    Serial.printf("Fader board: %d faders, touch on G5-G8\n", FADERS);
    calibrateNativeTouch();  // before the motors run, with no hands on the faders
  } else {
    Wire.begin(I2C_SDA, I2C_SCL);
    haveMpr = mpr.begin(0x5A, &Wire);
    Serial.printf("Fader board: %d faders, MPR121 %s\n", FADERS,
                  haveMpr ? "found" : "not found (detecting hands by motion)");
  }
  calRequest = true;  // calibrated by the motor task: from setup() the motors didn't move
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

void motorTask(void *) {
  uint32_t seen[MAX_FADERS] = {0};
  for (;;) {
    uint32_t now = millis();
    if (calRequest) {
      calibrate();
      calRequest = false;
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
        if (!motorsOff && t != f.target) {
          // A stream of targets (a linked channel or the app's fader being
          // moved): steer towards the latest one instead of restarting the
          // move each time, which made the fader lurch.
          bool stream = now - f.lastTargetAt < FOLLOW_MS;
          if (stream && f.enabled && !f.touched && t >= 0 && f.smooth >= 0) followTarget(n, t, now);
          else startMove(n, t);
          f.lastTargetAt = now;
        }
      }
      if (motorsOff && faders[n].moving) { faders[n].moving = false; coast(n); }
      updateFader(n, now);
    }
    vTaskDelay(1);
  }
}

void loop() {
  static uint32_t lastTouch = 0, lastReport = 0;
  uint32_t now = millis();
  readLink();
  while (Serial.available()) {  // DEBUG: "M <n>" motor test over USB
    static char cmd[16];
    static int clen = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[clen] = 0;
      int n;
      if (clen && cmd[0] == 'M' && sscanf(cmd + 1, "%d", &n) == 1 && n >= 0 && n <= FADERS) testRequest = n;
      if (clen && cmd[0] == 'C') calRequest = true;
      if (clen && cmd[0] == 'X') {
        motorsOff = !motorsOff;
        Serial.printf("motors %s\n", motorsOff ? "OFF" : "on");
      }
      clen = 0;
    } else if (clen < 15) {
      cmd[clen++] = c;
    }
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
    for (int n = 0; n < FADERS; n++)
      Serial.printf(" %s%.0f/%.0f", faders[n].touched ? "*" : "",
                    NATIVE_TOUCH && faders[n].touchOk ? (float)touchRead(TOUCH_PIN[n]) : 0.0f,
                    faders[n].touchBase);
    Serial.println();
  }
}
