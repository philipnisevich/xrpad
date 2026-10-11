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
//   Link to main board         TX G43 -> main G14, RX G44 <- main G13, GND
//
// Protocol (921600 baud, text lines):
//   main -> board  "T <n> <pos>"  move fader n (0-7) to pos 0-1000; pos < 0 = no target
//                  "C"            recalibrate all faders
//                  "?"            ask for the hello line
//   board -> main  "H <faders> <mpr121>"           hello (at boot and on "?")
//                  "S <touchmask> <p0> ... <p7>"   every 20 ms, positions 0-1000

#include <Wire.h>
#include <Adafruit_MPR121.h>

const int MAX_FADERS = 8;
const int FADERS = 8;  // how many are fitted (1 for testing with a single fader)

const uint8_t WIPER[MAX_FADERS] = {1, 2, 3, 4, 5, 6, 7, 8};
const uint8_t IN1[MAX_FADERS]   = {9, 10, 11, 12, 13, 14, 15, 16};   // PWM
const uint8_t IN2[MAX_FADERS]   = {17, 18, 21, 38, 39, 40, 41, 42};  // direction
const bool MOTOR_REVERSE[MAX_FADERS] = {false, false, false, false, false, false, false, false};
const bool POT_REVERSE[MAX_FADERS]   = {false, false, false, false, false, false, false, false};

const uint8_t I2C_SDA = 47, I2C_SCL = 48;
const uint8_t LINK_TX = 43, LINK_RX = 44;
const uint32_t LINK_BAUD = 921600;

const uint32_t MOTOR_PWM_HZ = 20000;  // above hearing
const uint8_t  MOTOR_PWM_BITS = 10;
const int      PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
const float    MOTOR_MIN_DUTY = 0.65f;
const float    MOTOR_KP = 5.0f;
const float    MOTOR_TOL = 0.008f;
const float    MOTOR_PROGRESS = 0.01f;   // getting this much closer counts as progress
const uint32_t MOTOR_BRAKE_MS = 30;
const uint32_t MOTOR_STALL_MS = 30;
const uint32_t MOTOR_KICK_MS = 50;
const uint32_t MOTOR_BREAKAWAY_MS = 60;
const uint32_t MOTOR_END_MS = 80;
const uint32_t MOTOR_TIMEOUT_MS = 1500;

const uint32_t CAL_DRIVE_MS = 500;
const float    CAL_END_GAP = 0.015f;    // -inf/+10 dB sit this far in from the end stops
const int      CAL_MIN_SPAN = 1800;     // raw ADC counts; less = calibration failed

const uint32_t TOUCH_READ_MS = 5;
const uint32_t TOUCH_HOLD_MS = 150;     // stay "touched" briefly after letting go
const uint32_t REPORT_MS = 20;

// Without an MPR121, a hand is detected by the fader moving while its motor
// isn't driving it.
const float    NOTOUCH_MOVE = 0.02f;
const uint32_t NOTOUCH_STILL_MS = 400;

struct Fader {
  float minRaw = 60, maxRaw = 4000;  // calibrated ends
  float smooth = -1;                 // position 0-1, filtered
  float target = -1;                 // -1 = no target
  bool moving = false, breakaway = false, touched = false;
  float startErr = 0, bestErr = 0;
  uint32_t startAt = 0, progressAt = 0, brakeUntil = 0, releasedAt = 0;
  float restPos = -1;                // no-MPR121 hand detection
  uint32_t movedAt = 0;
};

Fader faders[MAX_FADERS];
Adafruit_MPR121 mpr;
bool haveMpr = false;

// ---------- motor ----------

// IN1 carries PWM, IN2 the direction. Forward: IN2 low, IN1 = duty (drive /
// coast). Reverse: IN2 high, IN1 = 1 - duty (drive / brake). The control
// loop is closed, so the slightly different decay each way doesn't matter.
void drive(int n, float speed) {
  if (MOTOR_REVERSE[n]) speed = -speed;
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

float readPosition(int n) {
  Fader &f = faders[n];
  float raw = analogRead(WIPER[n]);
  float pos = constrain((raw - f.minRaw) / (f.maxRaw - f.minRaw), 0.0f, 1.0f);
  return POT_REVERSE[n] ? 1.0f - pos : pos;
}

void startMove(int n, float target) {
  Fader &f = faders[n];
  f.target = target;
  if (target < 0 || f.touched || f.smooth < 0) { f.moving = false; return; }
  f.moving = true;
  f.breakaway = true;
  f.startAt = f.progressAt = millis();
  f.startErr = f.bestErr = fabsf(target - f.smooth);
}

void updateFader(int n, uint32_t now) {
  Fader &f = faders[n];
  float pos = readPosition(n);
  f.smooth = f.smooth < 0 ? pos : f.smooth + 0.3f * (pos - f.smooth);

  if (f.brakeUntil && now >= f.brakeUntil) { coast(n); f.brakeUntil = 0; }
  if (!f.moving) return;

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
    brake(n);
    f.brakeUntil = now + MOTOR_BRAKE_MS;
    f.moving = false;
  } else if (now - f.startAt > 250 && fabsf(err) > f.startErr + 0.05f) {
    coast(n);
    f.moving = false;
    Serial.printf("fader %d: moving away from target, flip MOTOR_REVERSE[%d]\n", n + 1, n);
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

// Drive every fader into both end stops and record them.
void calibrate() {
  Serial.println("Calibrating faders");
  float lo[MAX_FADERS], hi[MAX_FADERS];
  for (int n = 0; n < FADERS; n++) drive(n, -1.0f);
  delay(CAL_DRIVE_MS);
  for (int n = 0; n < FADERS; n++) { lo[n] = averageRaw(n); coast(n); }
  delay(50);
  for (int n = 0; n < FADERS; n++) drive(n, 1.0f);
  delay(CAL_DRIVE_MS);
  for (int n = 0; n < FADERS; n++) { hi[n] = averageRaw(n); coast(n); }
  for (int n = 0; n < FADERS; n++) {
    Fader &f = faders[n];
    float a = min(lo[n], hi[n]), b = max(lo[n], hi[n]);
    if (b - a < CAL_MIN_SPAN) {
      Serial.printf("fader %d: calibration failed (%.0f-%.0f), check its wiring\n", n + 1, a, b);
      continue;
    }
    float gap = (b - a) * CAL_END_GAP;
    f.minRaw = a + gap;
    f.maxRaw = b - gap;
    f.smooth = -1;
    Serial.printf("fader %d: %.0f-%.0f\n", n + 1, f.minRaw, f.maxRaw);
  }
}

// ---------- touch ----------

void updateTouch(uint32_t now) {
  uint16_t mask = haveMpr ? mpr.touched() : 0;
  for (int n = 0; n < FADERS; n++) {
    Fader &f = faders[n];
    bool raw;
    if (haveMpr) {
      raw = mask & (1 << n);
    } else if (f.moving || f.brakeUntil || f.smooth < 0) {
      f.restPos = f.smooth;  // the motor is moving it, not a hand
      raw = false;
    } else {
      if (f.restPos < 0) f.restPos = f.smooth;
      if (fabsf(f.smooth - f.restPos) > NOTOUCH_MOVE) { f.restPos = f.smooth; f.movedAt = now; }
      raw = f.movedAt && now - f.movedAt < NOTOUCH_STILL_MS;
    }
    if (raw) {
      f.releasedAt = 0;
      if (!f.touched) {
        f.touched = true;
        if (f.moving) coast(n);
        f.moving = false;
      }
    } else if (f.touched) {
      if (!f.releasedAt) f.releasedAt = now;
      if (now - f.releasedAt >= TOUCH_HOLD_MS) f.touched = false;
    }
  }
}

// ---------- link ----------

void sendHello() { Serial1.printf("H %d %d\n", FADERS, haveMpr ? 1 : 0); }

void handleLine(char *line) {
  if (line[0] == 'T') {
    int n, pos;
    if (sscanf(line + 1, "%d %d", &n, &pos) == 2 && n >= 0 && n < FADERS)
      startMove(n, pos < 0 ? -1.0f : constrain(pos, 0, 1000) / 1000.0f);
  } else if (line[0] == 'C') {
    calibrate();
  } else if (line[0] == '?') {
    sendHello();
  }
}

void readLink() {
  static char buf[48];
  static int len = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
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
    int p = n < FADERS && faders[n].smooth >= 0 ? (int)lroundf(faders[n].smooth * 1000) : -1;
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
  Wire.begin(I2C_SDA, I2C_SCL);
  haveMpr = mpr.begin(0x5A, &Wire);
  Serial.printf("Fader board: %d faders, MPR121 %s\n", FADERS,
                haveMpr ? "found" : "not found (detecting hands by motion)");
  calibrate();
  sendHello();
}

void loop() {
  static uint32_t lastTick = 0, lastTouch = 0, lastReport = 0;
  uint32_t now = millis();
  readLink();
  if (now != lastTick) {  // motor control every millisecond
    lastTick = now;
    for (int n = 0; n < FADERS; n++) updateFader(n, now);
  }
  if (now - lastTouch >= TOUCH_READ_MS) { lastTouch = now; updateTouch(now); }
  if (now - lastReport >= REPORT_MS) { lastReport = now; report(); }
}
