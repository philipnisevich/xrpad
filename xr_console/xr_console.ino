// XR18 console — LCKFB ESP32-S3 core board (N8R8) over Wi-Fi/OSC.
//
// Turn the encoder to pick a channel (CH 1-16, AUX, MAIN). The motorized
// fader drives itself to that channel's level, and follows changes made from
// the app; moving it by hand sets the level. The OLED shows a compact
// readout, and the 4.0" TFT (ui.h) shows everything about the selected
// channel: level and live meter, EQ curve, gate/compressor, preamp, pan,
// sends, input history and an overview of all channels. Tap the TFT to mute,
// solo or select a channel.
//
// Wi-Fi is chosen on the TFT: at boot the board scans and joins the most
// recently used saved network that's in range, otherwise it shows a network
// picker (with an on-screen keyboard for passwords). The gear in the header
// opens settings to change network or recalibrate the touchscreen.
//
// If you grab the fader while the motor is moving it, the motor lets go and
// the fader falls back to pickup: it takes control once moved past the
// channel's level, with an arrow showing which way.
//
// Wiring (logic is 3V3, never 5V):
//   OLED (SSD1306 128x64 I2C)  VCC -> 3V3, GND -> GND, SDA -> G08, SCL -> G09
//   Encoder (KY-040 module)    +   -> 3V3, GND -> GND, CLK -> G06, DT -> G05
//   Slide fader (10k linear)   one end -> 3V3, other end -> GND, wiper -> G07,
//                              touch pin -> G10 (keep this wire short)
//   Motor driver (TC1508A)     + -> 5V, - -> GND, IN1 -> G12, IN2 -> G11,
//                              motor outputs (MOTOR-A) -> fader motor pins
//   TFT (4.0" ST7796 SPI)      VCC -> 5V, GND -> GND, CS -> G15, RESET -> G42,
//                              DC -> G41, SDI -> G17, SCK -> G18, LED -> G40,
//                              SDO -> G16, T_CLK -> G18, T_CS -> G39,
//                              T_DIN -> G17, T_DO -> G16, T_IRQ -> G38
//
// The fader only sends changes while the knob is being touched, so electrical
// noise on a fader that's sitting still never moves the mixer.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <algorithm>
#include "secrets.h"  // optional default network, used until one is saved

const uint8_t  ENC_CLK      = 6;
const uint8_t  ENC_DT       = 5;
const uint8_t  POT_PIN      = 7;      // must be ADC1 (G1-G10); ADC2 is unusable with Wi-Fi
const uint8_t  TOUCH_PIN    = 10;     // any of G1-G14 can sense touch
const bool     TOUCH_ENABLED = true;  // false = fader always live (no touch wire)
const uint8_t  OLED_SDA     = 8;
const uint8_t  OLED_SCL     = 9;
const bool     ENC_REVERSE  = false;  // flip if clockwise goes to the previous channel
const bool     POT_REVERSE  = false;  // flip if pushing the fader up lowers the level
const uint8_t  MOTOR_IN1    = 12;
const uint8_t  MOTOR_IN2    = 11;
const bool     MOTOR_REVERSE = false; // flip if the motor drives away from the target

const uint32_t POT_MIN_MV   = 40;     // bottom/top readings if boot calibration fails
const uint32_t POT_MAX_MV   = 3100;   // (ADC tops out around 3.1 V)
const float    POT_END_GAP  = 0.015f; // buffer at each end: -inf and +10 dB sit this
                                      // fraction of travel in from the end stops
const uint32_t CAL_DRIVE_MS = 500;    // full power into each end stop at boot
const uint32_t CAL_MIN_SPAN_MV = 1500; // less than this = calibration failed
const float    POT_DEADBAND = 5.0f / 1023.0f;  // ignore wobble smaller than this
const float    PICKUP_RANGE = 0.015f; // fader this close to the level = picked up
const uint32_t POT_READ_MS  = 5;
const float    TOUCH_ON     = 0.15f;  // reading this far above baseline = touched
const float    TOUCH_OFF    = 0.08f;  // ...and below this = released
                                      // (measured: idle +-0.2%, finger +170-330%)
const uint32_t TOUCH_READ_MS = 15;
const uint32_t TOUCH_HOLD_MS = 150;   // stay live briefly after letting go

const uint32_t MOTOR_PWM_HZ = 20000;  // above hearing, so the motor doesn't whine
const uint8_t  MOTOR_PWM_BITS = 10;
const float    MOTOR_MIN_DUTY = 0.65f; // least power that reliably keeps the fader moving on 5 V
const float    MOTOR_KP     = 5.0f;   // full power until within 20% of travel
const float    MOTOR_TOL    = 0.008f; // close enough to stop
const uint32_t MOTOR_BRAKE_MS = 30;   // short brake on arrival to stop coasting
const uint32_t MOTOR_STALL_MS = 30;   // no progress for this long = add power...
const float    MOTOR_PROGRESS = 0.01f; // getting this much closer counts as progress
                                      // (the reading jitters ~0.005 when still)
const uint32_t MOTOR_KICK_MS = 50;    // ...ramping to full power over this long
const uint32_t MOTOR_BREAKAWAY_MS = 60; // longest full-power start before giving way to the kick
const uint32_t MOTOR_END_MS = 80;     // -inf/+10 dB: stopped this long at the end stop = there
const uint32_t MOTOR_TIMEOUT_MS = 1500;

const uint16_t XR_PORT      = 10024;
const uint32_t XREMOTE_MS   = 8000;   // mixer drops subscriptions after 10 s
const uint32_t TIMEOUT_MS   = 20000;  // no packets for this long = lost mixer
const uint32_t QUERY_GAP_MS = 15;     // spacing between overview queries
const uint32_t DETAIL_GAP_MS = 8;     // spacing between selected-channel queries
const uint32_t SEND_GAP_MS  = 25;     // max fader update rate while moving
const uint32_t LOCK_MS      = 600;    // ignore mixer echoes this long after a move
const uint32_t FRAME_MS     = 33;

// ---------- channel strips ----------

struct Strip {
  char prefix[10];   // OSC prefix, e.g. "/ch/01"
  char label[7];     // shown in the header, e.g. "CH 1"
  char name[16];     // channel name from the mixer
  float fader;       // 0.0 - 1.0
  bool haveFader;
  bool nameKnown;    // name reply received (names can be empty)
  int8_t on;         // 1 = live, 0 = muted, -1 = unknown
  bool sendDue;      // local change not yet sent
  bool needsFinal;   // resend + confirm once the fader stops
  uint32_t lockedUntil;
};

const int NUM_STRIPS = 18;
const int AUX_STRIP = 16, MAIN_STRIP = 17;
Strip strips[NUM_STRIPS];
int selected = 0;

// Everything else the mixer reports about a strip. Unknown values are -1
// (ints) or NAN (floats), e.g. MAIN has no preamp and AUX no gate.
struct EqBand {
  int8_t type;       // 0 LCut, 1 LShv, 2 PEQ, 3 VEQ, 4 HShv, 5 HCut
  float f, g, q;     // raw 0.0 - 1.0 values as the mixer sends them
};

struct Detail {
  int8_t color, lr, phantom, hpOn, invert, gateOn, dynOn, dynRatio, eqOn, solo;
  float pan, gain, hpf, gateThr, dynThr;
  EqBand eq[4];
  float sends[10];   // bus 1-6, FX 1-4
};

Detail details[NUM_STRIPS];

// Live input meters (dBFS) and a scrolling history of each strip.
const int HIST_LEN = 220;
float meterDb[NUM_STRIPS];
int8_t hist[NUM_STRIPS][HIST_LEN];
uint16_t histHead = 0;
uint32_t meterSeq = 0;

void resetDetail(Detail &d) {
  d.color = d.lr = d.phantom = d.hpOn = d.invert = d.gateOn = d.dynOn = -1;
  d.dynRatio = d.eqOn = d.solo = -1;
  d.pan = d.gain = d.hpf = d.gateThr = d.dynThr = NAN;
  for (EqBand &b : d.eq) { b.type = -1; b.f = b.g = b.q = NAN; }
  for (float &s : d.sends) s = NAN;
}

void initStrips() {
  for (int i = 0; i < 16; i++) {
    snprintf(strips[i].prefix, sizeof(strips[i].prefix), "/ch/%02d", i + 1);
    snprintf(strips[i].label, sizeof(strips[i].label), "CH %d", i + 1);
  }
  strcpy(strips[AUX_STRIP].prefix, "/rtn/aux");
  strcpy(strips[AUX_STRIP].label, "AUX");
  strcpy(strips[MAIN_STRIP].prefix, "/lr");
  strcpy(strips[MAIN_STRIP].label, "MAIN");
  for (int i = 0; i < NUM_STRIPS; i++) {
    strips[i].on = -1;
    resetDetail(details[i]);
    meterDb[i] = NAN;
    memset(hist[i], -128, HIST_LEN);
  }
}

// X-Air fader law: 0.0-1.0 maps to -inf..+10 dB in four linear segments.
float faderToDb(float f) {
  if (f >= 0.5f)    return f * 40.0f - 30.0f;
  if (f >= 0.25f)   return f * 80.0f - 50.0f;
  if (f >= 0.0625f) return f * 160.0f - 70.0f;
  return f * 480.0f - 90.0f;
}

float dbToFader(float db) {
  if (db >= -10.0f) return (db + 30.0f) / 40.0f;
  if (db >= -30.0f) return (db + 50.0f) / 80.0f;
  if (db >= -60.0f) return (db + 70.0f) / 160.0f;
  return max((db + 90.0f) / 480.0f, 0.0f);
}

// Shared with the TFT task: anything it reads is only changed while holding
// this lock.
SemaphoreHandle_t stateLock;

// Touch actions from the TFT, handled by the main loop (which owns the network).
enum UiActionType : uint8_t { UI_SELECT, UI_MUTE, UI_SOLO, UI_SCAN };
struct UiAction { UiActionType type; int8_t arg; };
QueueHandle_t uiQueue;
void handleUiAction(const UiAction &a);

// ---------- network state ----------

WiFiUDP udp;
IPAddress mixerIp;
volatile bool haveMixer = false;
uint32_t lastRx = 0, lastXremote = 0, lastProbe = 0, lastQuery = 0, lastSend = 0;
int sweepPos = -1;   // position in the all-strips query sweep, -1 = idle
bool sweepRepair = false;  // this pass only asks for what's still missing
int sweepRepairs = 0;
int detailPos = -1;  // position in the selected-strip query list, -1 = idle
int detailStrip = 0;
bool detailDone = false;     // every detail query for the selected strip sent
uint32_t syncStart = 0;      // when the current mixer was found (0 = none)
char mixerName[24] = "", mixerModel[16] = "", mixerFw[16] = "";
bool dirty = true;   // OLED needs redrawing
volatile int bootStage = 0;  // 0 calibrating fader, 1 running

// ---------- Wi-Fi ----------

// Not connected and waiting for a choice / joining / joined / join failed.
enum NetState : uint8_t { NET_IDLE, NET_CONNECTING, NET_CONNECTED, NET_FAILED };
const int MAX_SCAN = 24, MAX_SAVED = 5;
const uint32_t JOIN_TIMEOUT_MS = 15000;

struct ScanEntry { char ssid[33]; int8_t rssi; bool open; };
struct SavedNet { char ssid[33]; char pass[65]; };

volatile NetState netState = NET_IDLE;
char netSsid[33] = "";           // network being joined or joined
char joinPass[65] = "";
uint32_t joinStart = 0;
ScanEntry scanList[MAX_SCAN];
int scanCount = 0;
uint32_t scanSeq = 0;            // bumped when new results arrive
volatile bool scanning = false;
bool autoJoinPending = false;    // boot: join a saved network if one is in range
SavedNet savedNets[MAX_SAVED];   // most recently used first
int savedCount = 0;

// Set by the TFT task (under stateLock), acted on by the main loop.
struct JoinRequest { bool pending, useSaved; char ssid[33]; char pass[65]; } joinReq;
const SavedNet *findSaved(const char *ssid);

// ---------- slide fader state ----------

float potPos = 0.0f;      // fader position, 0.0 - 1.0, on the mixer's 1023-step grid
bool pickedUp = false;    // fader is in control of the selected channel
volatile bool touched = !TOUCH_ENABLED;
float touchBaseline = 0.0f;

// ---------- motor ----------

// The motor runs in its own task so it's controlled every millisecond; the
// main loop stalls ~35 ms on each OLED redraw, long enough for the motor to
// overshoot badly. The task also owns the ADC and publishes the filtered
// fader position for the main loop.

volatile float potSmooth = -1.0f;   // fader position 0.0 - 1.0, filtered
float potMinMv = POT_MIN_MV, potMaxMv = POT_MAX_MV;  // set by calibrateFader()
volatile float motorTarget = 0.0f;
volatile uint32_t motorSeq = 0;     // bumped for each new move request
volatile bool motorArrived = false;

void motorDrive(float speed) {  // -1.0 (down) .. 1.0 (up), 0 = coast
  if (MOTOR_REVERSE) speed = -speed;
  uint32_t duty = (uint32_t)(fabsf(speed) * ((1 << MOTOR_PWM_BITS) - 1));
  ledcWrite(MOTOR_IN1, speed > 0.0f ? duty : 0);
  ledcWrite(MOTOR_IN2, speed < 0.0f ? duty : 0);
}

void motorBrake() {  // both inputs high shorts the motor
  ledcWrite(MOTOR_IN1, (1 << MOTOR_PWM_BITS) - 1);
  ledcWrite(MOTOR_IN2, (1 << MOTOR_PWM_BITS) - 1);
}

void motorTask(void *) {
  float smooth = -1.0f, startErr = 0.0f, bestErr = 0.0f;
  bool moving = false, breakaway = false;
  uint32_t seenSeq = 0, startAt = 0, brakeUntil = 0, progressAt = 0;
  // Debug trace of slow moves: position and power every 10 ms.
  static int16_t tracePos[160];
  static int8_t tracePwr[160];
  int traceN = 0;
  uint32_t traceAt = 0;
  float tracePower = 0.0f;
  bool wasMoving = false;

  for (;;) {
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(POT_PIN);
    mv /= 4;
    float raw = constrain(((float)mv - potMinMv) / (potMaxMv - potMinMv), 0.0f, 1.0f);
    if (POT_REVERSE) raw = 1.0f - raw;
    smooth = smooth < 0.0f ? raw : smooth + 0.3f * (raw - smooth);
    potSmooth = smooth;

    uint32_t now = millis();
    if (brakeUntil && now >= brakeUntil) {
      motorDrive(0.0f);
      brakeUntil = 0;
    }

    if (motorSeq != seenSeq) {  // new target
      seenSeq = motorSeq;
      moving = !touched;
      startAt = now;
      startErr = fabsf(motorTarget - smooth);
      bestErr = startErr;
      progressAt = now;
      breakaway = true;
      traceN = 0;
      traceAt = 0;
    }

    if (moving) {
      float err = motorTarget - smooth;
      // -inf and +10 dB aren't aimed at: the motor drives into the end stop
      // at full power until the fader stops, so it always gets all the way.
      bool endTarget = motorTarget <= 0.0f || motorTarget >= 1.0f;
      // Progress = actually getting closer to the target; position noise
      // alone mustn't count, or a stalled fader looks like it's moving.
      if (fabsf(err) < bestErr - MOTOR_PROGRESS) {
        bestErr = fabsf(err);
        progressAt = now;
        breakaway = false;  // it's moving: static friction is beaten
      }
      if (now - startAt > MOTOR_BREAKAWAY_MS) breakaway = false;
      uint32_t stuck = now - progressAt;

      if (touched) {
        motorDrive(0.0f);
        moving = false;
        Serial.printf("[%lu] motor: hand on fader, letting go\n", now);
      } else if (endTarget && stuck > MOTOR_END_MS) {
        motorDrive(0.0f);
        moving = false;
        motorArrived = true;
        Serial.printf("[%lu] motor: at end stop in %lu ms (%.3f)\n", now, now - startAt, smooth);
      } else if (!endTarget && fabsf(err) <= MOTOR_TOL) {
        motorBrake();
        brakeUntil = now + MOTOR_BRAKE_MS;
        moving = false;
        motorArrived = true;
        Serial.printf("[%lu] motor: arrived in %lu ms\n", now, now - startAt);
      } else if (now - startAt > 250 && fabsf(err) > startErr + 0.05f) {
        motorDrive(0.0f);
        moving = false;
        Serial.printf("[%lu] motor: moving away from target, set MOTOR_REVERSE\n", now);
      } else if (now - startAt > MOTOR_TIMEOUT_MS) {
        motorDrive(0.0f);
        moving = false;
        Serial.printf("[%lu] motor: gave up, %.3f from target\n", now, err);
      } else {
        // A resting fader needs more force to get going than to keep moving,
        // so every move starts at full power until the fader shifts. After
        // that, if it stalls on low power near the target, ramp back up.
        float kick = stuck > MOTOR_STALL_MS ? (float)(stuck - MOTOR_STALL_MS) / MOTOR_KICK_MS : 0.0f;
        float power = constrain(fabsf(err) * MOTOR_KP, MOTOR_MIN_DUTY, 1.0f);
        power = endTarget || breakaway ? 1.0f : min(power + kick, 1.0f);
        tracePower = err > 0.0f ? power : -power;
        motorDrive(tracePower);
      }
      if (moving && now - traceAt >= 10 && traceN < 160) {
        traceAt = now;
        tracePos[traceN] = (int16_t)lroundf(smooth * 1000.0f);
        tracePwr[traceN] = (int8_t)lroundf(tracePower * 100.0f);
        traceN++;
      }
    }
    if (wasMoving && !moving && now - startAt > 120) {
      Serial.printf("[%lu] trace to %.3f, pos x1000 / power %% every 10 ms:\n", now, motorTarget);
      for (int i = 0; i < traceN; i++) Serial.printf("%d/%d ", tracePos[i], tracePwr[i]);
      Serial.println();
    }
    wasMoving = moving;
    vTaskDelay(1);
  }
}

uint32_t readEndStop(float speed) {
  motorDrive(speed);
  delay(CAL_DRIVE_MS);
  uint32_t mv = 0;
  for (int i = 0; i < 32; i++) mv += analogReadMilliVolts(POT_PIN);
  motorDrive(0.0f);
  delay(50);
  return mv / 32;
}

// Drive into both end stops and measure them, so -inf and +10 dB map to
// points the motor can always reach, a small gap in from each end.
void calibrateFader() {
  uint32_t a = readEndStop(-1.0f), b = readEndStop(1.0f);
  uint32_t bottom = min(a, b), top = max(a, b);  // either way round if POT_REVERSE
  if (top < bottom + CAL_MIN_SPAN_MV) {
    Serial.printf("Fader calibration failed (bottom %lu mV, top %lu mV), using defaults\n",
                  bottom, top);
    return;
  }
  float gap = (top - bottom) * POT_END_GAP;
  potMinMv = bottom + gap;
  potMaxMv = top - gap;
  Serial.printf("Fader calibrated: ends %lu-%lu mV, using %.0f-%.0f mV\n",
                bottom, top, potMinMv, potMaxMv);
}

void motorBegin() {
  ledcAttach(MOTOR_IN1, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(MOTOR_IN2, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  motorDrive(0.0f);
  calibrateFader();
  xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 3, nullptr, 1);
}

// Drive the fader to the selected channel's level.
void requestMove() {
  const Strip &s = strips[selected];
  if (!s.haveFader || potSmooth < 0.0f) return;
  if (fabsf(potSmooth - s.fader) <= MOTOR_TOL) {
    pickedUp = true;
    potPos = s.fader;
    return;
  }
  pickedUp = false;
  motorTarget = s.fader;
  motorSeq = motorSeq + 1;
}

// Forget everything read from the mixer, so a new (or re-found) mixer is
// loaded from scratch and the TFT shows the sync screen again.
void resetMixerState() {
  for (int i = 0; i < NUM_STRIPS; i++) {
    strips[i].haveFader = strips[i].nameKnown = false;
    strips[i].on = -1;
    resetDetail(details[i]);
    meterDb[i] = NAN;
  }
  detailDone = false;
  syncStart = 0;
  pickedUp = false;
}

// ---------- Wi-Fi networks ----------

void loadSavedNetworks() {
  Preferences prefs;
  prefs.begin("xrnet", true);
  savedCount = min((int)prefs.getUChar("n", 0), MAX_SAVED);
  for (int i = 0; i < savedCount; i++) {
    char key[4];
    snprintf(key, sizeof(key), "s%d", i);
    prefs.getString(key, savedNets[i].ssid, sizeof(savedNets[i].ssid));
    snprintf(key, sizeof(key), "p%d", i);
    prefs.getString(key, savedNets[i].pass, sizeof(savedNets[i].pass));
  }
  prefs.end();
  // First run: start from the network in secrets.h, if one was filled in.
  if (savedCount == 0 && strlen(WIFI_SSID) && strcmp(WIFI_SSID, "your-wifi-name") != 0) {
    strlcpy(savedNets[0].ssid, WIFI_SSID, sizeof(savedNets[0].ssid));
    strlcpy(savedNets[0].pass, WIFI_PASSWORD, sizeof(savedNets[0].pass));
    savedCount = 1;
  }
}

// Moves (or adds) a network to the front of the saved list.
void rememberNetwork(const char *ssid, const char *pass) {
  int found = -1;
  for (int i = 0; i < savedCount; i++)
    if (!strcmp(savedNets[i].ssid, ssid)) found = i;
  int last = found >= 0 ? found : min(savedCount, MAX_SAVED - 1);
  for (int i = last; i > 0; i--) savedNets[i] = savedNets[i - 1];
  strlcpy(savedNets[0].ssid, ssid, sizeof(savedNets[0].ssid));
  strlcpy(savedNets[0].pass, pass, sizeof(savedNets[0].pass));
  if (found < 0 && savedCount < MAX_SAVED) savedCount++;

  Preferences prefs;
  prefs.begin("xrnet", false);
  prefs.putUChar("n", savedCount);
  for (int i = 0; i < savedCount; i++) {
    char key[4];
    snprintf(key, sizeof(key), "s%d", i);
    prefs.putString(key, savedNets[i].ssid);
    snprintf(key, sizeof(key), "p%d", i);
    prefs.putString(key, savedNets[i].pass);
  }
  prefs.end();
}

const SavedNet *findSaved(const char *ssid) {
  for (int i = 0; i < savedCount; i++)
    if (!strcmp(savedNets[i].ssid, ssid)) return &savedNets[i];
  return nullptr;
}

void startScan() {
  if (scanning) return;
  scanning = true;
  WiFi.scanNetworks(true);  // async; results collected in netService()
}

void startJoin(const char *ssid, const char *pass) {
  strlcpy(netSsid, ssid, sizeof(netSsid));
  strlcpy(joinPass, pass, sizeof(joinPass));
  WiFi.disconnect();
  WiFi.begin(netSsid, joinPass[0] ? joinPass : nullptr);
  netState = NET_CONNECTING;
  joinStart = millis();
  haveMixer = false;
  resetMixerState();
  lastRx = 0;
  dirty = true;
  Serial.printf("Joining %s\n", netSsid);
}

// Called by the TFT task.
void requestJoin(const char *ssid, const char *pass, bool useSaved) {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  strlcpy(joinReq.ssid, ssid, sizeof(joinReq.ssid));
  strlcpy(joinReq.pass, pass, sizeof(joinReq.pass));
  joinReq.useSaved = useSaved;
  joinReq.pending = true;
  xSemaphoreGive(stateLock);
}

void collectScan() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  scanning = false;
  if (n < 0) {  // failed: try again
    startScan();
    return;
  }
  scanCount = 0;
  for (int i = 0; i < n; i++) {  // strongest first, one entry per name
    String ssid = WiFi.SSID(i);
    if (!ssid.length()) continue;  // hidden network
    int dup = -1;
    for (int j = 0; j < scanCount; j++)
      if (ssid == scanList[j].ssid) dup = j;
    int8_t rssi = WiFi.RSSI(i);
    if (dup >= 0) {
      scanList[dup].rssi = max(scanList[dup].rssi, rssi);
      continue;
    }
    if (scanCount == MAX_SCAN) continue;
    ScanEntry &e = scanList[scanCount++];
    strlcpy(e.ssid, ssid.c_str(), sizeof(e.ssid));
    e.rssi = rssi;
    e.open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
  }
  WiFi.scanDelete();
  std::sort(scanList, scanList + scanCount,
            [](const ScanEntry &a, const ScanEntry &b) { return a.rssi > b.rssi; });
  scanSeq++;
  Serial.printf("Scan found %d networks\n", scanCount);

  if (autoJoinPending) {  // boot: most recently used saved network in range
    autoJoinPending = false;
    for (int i = 0; i < savedCount; i++)
      for (int j = 0; j < scanCount; j++)
        if (!strcmp(savedNets[i].ssid, scanList[j].ssid)) {
          startJoin(savedNets[i].ssid, savedNets[i].pass);
          return;
        }
  }
}

void netService() {
  if (scanning) collectScan();

  if (joinReq.pending && !scanning) {
    joinReq.pending = false;
    const SavedNet *sv = joinReq.useSaved ? findSaved(joinReq.ssid) : nullptr;
    startJoin(joinReq.ssid, sv ? sv->pass : joinReq.pass);
  }

  if (netState == NET_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      netState = NET_CONNECTED;
      rememberNetwork(netSsid, joinPass);
      udp.stop();
      udp.begin(XR_PORT);
      lastXremote = 0;
      dirty = true;
      Serial.printf("Joined %s, my IP is %s\n", netSsid, WiFi.localIP().toString().c_str());
      if (strlen(XR18_IP) && mixerIp.fromString(XR18_IP)) {
        haveMixer = true;
        syncStart = millis();
      }
    } else if (millis() - joinStart > JOIN_TIMEOUT_MS) {  // not `now`: joinStart may be newer
      WiFi.disconnect();
      netState = NET_FAILED;
      dirty = true;
      Serial.printf("Couldn't join %s\n", netSsid);
      startScan();
    }
  }
}

// ---------- OSC encoding ----------

size_t padTo4(size_t n) { return (n + 4) & ~3u; }  // includes the null terminator

size_t oscString(uint8_t *buf, const char *s) {
  size_t len = strlen(s), padded = padTo4(len);
  memset(buf, 0, padded);
  memcpy(buf, s, len);
  return padded;
}

void sendPacket(IPAddress ip, const uint8_t *buf, size_t n) {
  udp.beginPacket(ip, XR_PORT);
  udp.write(buf, n);
  udp.endPacket();
}

// type 0 = no argument, 'i' = int32, 'f' = float (raw holds the bits)
void sendOsc(IPAddress ip, const char *address, char type = 0, uint32_t raw = 0) {
  uint8_t buf[64];
  size_t n = oscString(buf, address);
  if (type) {
    char tags[3] = {',', type, 0};
    n += oscString(buf + n, tags);
    buf[n++] = raw >> 24; buf[n++] = raw >> 16; buf[n++] = raw >> 8; buf[n++] = raw;
  }
  sendPacket(ip, buf, n);
}

void sendOscString(IPAddress ip, const char *address, const char *value) {
  uint8_t buf[96];
  size_t n = oscString(buf, address);
  n += oscString(buf + n, ",s");
  n += oscString(buf + n, value);
  sendPacket(ip, buf, n);
}

void sendStripOsc(const Strip &s, const char *suffix, bool withFader = false) {
  char address[32];
  snprintf(address, sizeof(address), "%s%s", s.prefix, suffix);
  uint32_t raw = 0;
  memcpy(&raw, &s.fader, 4);
  sendOsc(mixerIp, address, withFader ? 'f' : 0, raw);
}

// ---------- OSC decoding ----------

uint32_t readBE32(const uint8_t *p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

// Reads a single int or float argument as a float.
bool readNumber(const char *tags, const uint8_t *arg, size_t argLen, float &out) {
  if (argLen < 4) return false;
  uint32_t raw = readBE32(arg);
  if (strcmp(tags, ",i") == 0) { out = (float)(int32_t)raw; return true; }
  if (strcmp(tags, ",f") == 0) { memcpy(&out, &raw, 4); return true; }
  return false;
}

// /meters/1: a blob of 40 little-endian int16 levels in 1/256 dB:
// 16 channels, aux L/R, 4 FX returns (L/R), 6 buses, 4 FX sends, main L/R,
// monitor L/R.
void handleMeters(const uint8_t *arg, size_t argLen) {
  if (argLen < 8) return;
  uint32_t blobLen = readBE32(arg);
  const uint8_t *blob = arg + 4;
  if (blobLen < 4 || blobLen + 4 > argLen) return;
  uint32_t count = blob[0] | blob[1] << 8 | blob[2] << 16 | (uint32_t)blob[3] << 24;
  if (count < 38 || 4 + count * 2 > blobLen) return;
  auto level = [&](int i) {
    int16_t v = (int16_t)(blob[4 + i * 2] | blob[5 + i * 2] << 8);
    return max(v / 256.0f, -90.0f);
  };
  for (int i = 0; i < 16; i++) meterDb[i] = level(i);
  meterDb[AUX_STRIP] = max(level(16), level(17));
  meterDb[MAIN_STRIP] = max(level(36), level(37));
  for (int i = 0; i < NUM_STRIPS; i++) hist[i][histHead] = (int8_t)max(meterDb[i], -127.0f);
  histHead = (histHead + 1) % HIST_LEN;
  meterSeq++;
}

// Map /headamp/NN and /-stat/solosw/NN numbers to strips (17 = aux).
int stripForInput(int n) {
  if (n >= 1 && n <= 16) return n - 1;
  if (n == 17) return AUX_STRIP;
  return -1;
}

void applyDetail(int i, const char *suffix, const char *tags, const uint8_t *arg, size_t argLen) {
  Detail &d = details[i];
  float v;
  if (!readNumber(tags, arg, argLen, v)) return;
  int iv = (int)v;

  if      (!strcmp(suffix, "/mix/on"))        { strips[i].on = iv; dirty = true; }
  else if (!strcmp(suffix, "/config/color"))  d.color = iv;
  else if (!strcmp(suffix, "/mix/pan"))       d.pan = v;
  else if (!strcmp(suffix, "/mix/lr"))        d.lr = iv;
  else if (!strcmp(suffix, "/preamp/hpon"))   d.hpOn = iv;
  else if (!strcmp(suffix, "/preamp/hpf"))    d.hpf = v;
  else if (!strcmp(suffix, "/preamp/invert")) d.invert = iv;
  else if (!strcmp(suffix, "/gate/on"))       d.gateOn = iv;
  else if (!strcmp(suffix, "/gate/thr"))      d.gateThr = v;
  else if (!strcmp(suffix, "/dyn/on"))        d.dynOn = iv;
  else if (!strcmp(suffix, "/dyn/thr"))       d.dynThr = v;
  else if (!strcmp(suffix, "/dyn/ratio"))     d.dynRatio = iv;
  else if (!strcmp(suffix, "/eq/on"))         d.eqOn = iv;
  else if (!strncmp(suffix, "/eq/", 4) && suffix[4] >= '1' && suffix[4] <= '4' && suffix[5] == '/') {
    EqBand &b = d.eq[suffix[4] - '1'];
    const char *p = suffix + 6;
    if      (!strcmp(p, "type")) b.type = iv;
    else if (!strcmp(p, "f"))    b.f = v;
    else if (!strcmp(p, "g"))    b.g = v;
    else if (!strcmp(p, "q"))    b.q = v;
  } else if (!strncmp(suffix, "/mix/", 5) && !strcmp(suffix + 7, "/level")) {
    int n = atoi(suffix + 5);
    if (n >= 1 && n <= 10) d.sends[n - 1] = v;
  }
}

void handlePacket(const uint8_t *buf, size_t len, IPAddress from) {
  if (len < 4) return;
  const char *address = (const char *)buf;
  size_t pos = padTo4(strnlen(address, len));
  if (pos >= len || buf[pos] != ',') return;
  const char *tags = (const char *)buf + pos;
  pos += padTo4(strnlen(tags, len - pos));
  const uint8_t *arg = buf + pos;
  size_t argLen = pos < len ? len - pos : 0;

  if (strcmp(address, "/xinfo") == 0) {
    // Reply args: ip, name, model, firmware
    const char *field[4] = {"", "", "", ""};
    size_t p = 0;
    for (int f = 0; f < 4 && p < argLen; f++) {
      field[f] = (const char *)arg + p;
      p += padTo4(strnlen(field[f], argLen - p));
    }
    strlcpy(mixerName, field[1], sizeof(mixerName));
    strlcpy(mixerModel, field[2], sizeof(mixerModel));
    strlcpy(mixerFw, field[3], sizeof(mixerFw));
    if (!haveMixer) {
      mixerIp = from;
      haveMixer = true;
      syncStart = millis();
      lastXremote = 0;  // subscribe and start a query sweep right away
      dirty = true;
      Serial.printf("Found mixer at %s\n", from.toString().c_str());
      Serial.printf("Fader range %.0f-%.0f mV\n", potMinMv, potMaxMv);
    }
    return;
  }

  if (strcmp(address, "/meters/1") == 0 && strcmp(tags, ",b") == 0) {
    handleMeters(arg, argLen);
    return;
  }

  float v;
  if (strncmp(address, "/headamp/", 9) == 0) {
    int i = stripForInput(atoi(address + 9));
    if (i < 0 || !readNumber(tags, arg, argLen, v)) return;
    const char *suffix = address + 11;
    if (!strcmp(suffix, "/gain")) details[i].gain = v;
    else if (!strcmp(suffix, "/phantom")) details[i].phantom = (int)v;
    return;
  }
  if (strncmp(address, "/-stat/solosw/", 14) == 0) {
    int i = stripForInput(atoi(address + 14));
    if (i >= 0 && readNumber(tags, arg, argLen, v)) details[i].solo = (int)v;
    return;
  }

  for (int i = 0; i < NUM_STRIPS; i++) {
    Strip &s = strips[i];
    size_t plen = strlen(s.prefix);
    if (strncmp(address, s.prefix, plen) != 0 || address[plen] != '/') continue;
    const char *suffix = address + plen;

    if (strcmp(suffix, "/mix/fader") == 0 && strcmp(tags, ",f") == 0 && argLen >= 4) {
      if (millis() < s.lockedUntil) return;  // we're moving it; ignore echoes
      uint32_t raw = readBE32(arg);
      float f;
      memcpy(&f, &raw, 4);
      bool changed = !s.haveFader || f != s.fader;
      if (changed) dirty = true;
      s.fader = f;
      s.haveFader = true;
      // Moved from the app or a scene: drive the fader there, or if a hand
      // is on it, make the hand pick it up again.
      if (i == selected && changed) {
        if (touched) {
          if (fabsf(f - potPos) > PICKUP_RANGE) pickedUp = false;
        } else {
          requestMove();
        }
      }
    } else if (strcmp(suffix, "/config/name") == 0 && strcmp(tags, ",s") == 0 && argLen > 0) {
      size_t n = strnlen((const char *)arg, min(argLen, sizeof(s.name) - 1));
      if (strncmp(s.name, (const char *)arg, n) != 0 || s.name[n]) dirty = true;
      memcpy(s.name, arg, n);
      s.name[n] = 0;
      s.nameKnown = true;
    } else {
      applyDetail(i, suffix, tags, arg, argLen);
    }
    return;
  }
}

// ---------- state queries ----------

// Everything the TFT shows about one strip. Returns false past the end.
bool detailQuery(int strip, int idx, char *addr, size_t size) {
  static const char *const FIXED[] = {
    "/config/color", "/mix/on", "/mix/pan", "/mix/lr", "/preamp/hpon", "/preamp/hpf",
    "/preamp/invert", "/gate/on", "/gate/thr", "/dyn/on", "/dyn/thr", "/dyn/ratio", "/eq/on",
  };
  static const char *const EQ_PARAMS[] = {"type", "f", "g", "q"};
  const int nFixed = sizeof(FIXED) / sizeof(FIXED[0]);
  const char *prefix = strips[strip].prefix;

  if (idx < nFixed) { snprintf(addr, size, "%s%s", prefix, FIXED[idx]); return true; }
  idx -= nFixed;
  if (idx < 16) { snprintf(addr, size, "%s/eq/%d/%s", prefix, idx / 4 + 1, EQ_PARAMS[idx % 4]); return true; }
  idx -= 16;
  if (idx < 10) { snprintf(addr, size, "%s/mix/%02d/level", prefix, idx + 1); return true; }
  idx -= 10;
  int input = strip == AUX_STRIP ? 17 : strip < 16 ? strip + 1 : 0;
  if (!input) return false;  // MAIN has no preamp or solo switch here
  if (idx == 0) { snprintf(addr, size, "/headamp/%02d/gain", input); return true; }
  if (idx == 1) { snprintf(addr, size, "/headamp/%02d/phantom", input); return true; }
  if (idx == 2) { snprintf(addr, size, "/-stat/solosw/%02d", input); return true; }
  return false;
}

int detailQueryCount(int strip) {
  char addr[40];
  int n = 0;
  while (detailQuery(strip, n, addr, sizeof(addr))) n++;
  return n;
}

void runQueries(uint32_t now) {
  // The selected strip's details come first so the TFT fills in quickly.
  if (detailPos >= 0) {
    if (now - lastQuery < DETAIL_GAP_MS) return;
    char addr[40];
    if (detailQuery(detailStrip, detailPos++, addr, sizeof(addr))) {
      lastQuery = now;
      sendOsc(mixerIp, addr);
    } else {
      detailPos = -1;
      if (detailStrip == selected) detailDone = true;
    }
    return;
  }
  // Then names, faders and mutes of every strip, for the overview. UDP
  // replies sometimes get lost, so after a full pass, extra passes ask again
  // for only what's still missing.
  if (sweepPos >= 0 && now - lastQuery >= QUERY_GAP_MS) {
    static const char *const SWEEP[] = {"/config/name", "/mix/fader", "/mix/on"};
    auto known = [](int pos) {
      const Strip &s = strips[pos / 3];
      return pos % 3 == 0 ? s.nameKnown : pos % 3 == 1 ? s.haveFader : s.on >= 0;
    };
    while (sweepRepair && sweepPos < NUM_STRIPS * 3 && known(sweepPos)) sweepPos++;
    if (sweepPos < NUM_STRIPS * 3) {
      lastQuery = now;
      sendStripOsc(strips[sweepPos / 3], SWEEP[sweepPos % 3]);
      sweepPos++;
    }
    if (sweepPos >= NUM_STRIPS * 3) {
      bool missing = false;
      for (int p = 0; p < NUM_STRIPS * 3; p++) missing |= !known(p);
      if (missing && sweepRepairs < 5) {
        sweepRepairs++;
        sweepRepair = true;
        sweepPos = 0;
      } else {
        sweepPos = -1;
      }
    }
  }
}

// ---------- encoder ----------

// The encoder makes 4 edges per click and rests with both pins high. Clicks
// are counted in the interrupt when it settles back at rest, so a partial
// wiggle or bounce never counts.
volatile int8_t encEdges = 0;
volatile uint8_t encState = 0b11;
volatile int32_t encClicks = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEncoder() {
  static const int8_t TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  uint8_t ab = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  portENTER_CRITICAL_ISR(&encMux);
  encState = ((encState << 2) | ab) & 0x0F;
  encEdges += TABLE[encState];
  if (ab == 0b11) {  // back at rest
    if (encEdges >= 2 || encEdges <= -2) encClicks += encEdges > 0 ? 1 : -1;
    encEdges = 0;
  }
  portEXIT_CRITICAL_ISR(&encMux);
}

int takeClicks() {
  portENTER_CRITICAL(&encMux);
  int clicks = encClicks;
  encClicks = 0;
  portEXIT_CRITICAL(&encMux);
  return ENC_REVERSE ? -clicks : clicks;
}

void selectStrip(int i) {
  selected = (i % NUM_STRIPS + NUM_STRIPS) % NUM_STRIPS;
  const Strip &s = strips[selected];
  pickedUp = s.haveFader && fabsf(potPos - s.fader) <= PICKUP_RANGE;
  if (!touched) requestMove();
  detailStrip = selected;  // refresh everything the TFT shows
  detailPos = 0;
  detailDone = false;
  dirty = true;
  Serial.printf("[%lu] select %s \"%s\"\n", millis(), s.label, s.name);
}

void handleUiAction(const UiAction &a) {
  if (a.type == UI_SELECT) {
    selectStrip(a.arg);
    return;
  }
  if (a.type == UI_SCAN) {
    startScan();
    return;
  }
  if (!haveMixer) return;
  char addr[32];
  if (a.type == UI_MUTE) {
    Strip &s = strips[selected];
    if (s.on < 0) return;
    s.on = s.on ? 0 : 1;
    snprintf(addr, sizeof(addr), "%s/mix/on", s.prefix);
    sendOsc(mixerIp, addr, 'i', s.on);
    dirty = true;
  } else if (a.type == UI_SOLO) {
    int input = selected == AUX_STRIP ? 17 : selected < 16 ? selected + 1 : 0;
    Detail &d = details[selected];
    if (!input) return;
    d.solo = d.solo == 1 ? 0 : 1;
    snprintf(addr, sizeof(addr), "/-stat/solosw/%02d", input);
    sendOsc(mixerIp, addr, 'i', d.solo);
  }
}

// ---------- slide fader ----------

// Returns true when the fader moved by more than the deadband.
bool readPot() {
  float smooth = potSmooth;  // filtered by the motor task, which owns the ADC
  if (smooth < 0.0f) return false;

  float pos;
  if (smooth < 0.01f)      pos = 0.0f;  // snap the ends so -inf and +10 dB
  else if (smooth > 0.99f) pos = 1.0f;  // are always reachable
  else if (smooth > potPos + POT_DEADBAND) pos = smooth - POT_DEADBAND;
  else if (smooth < potPos - POT_DEADBAND) pos = smooth + POT_DEADBAND;
  else return false;  // within the deadband: hold still
  pos = roundf(pos * 1023.0f) / 1023.0f;
  if (pos == potPos) return false;
  potPos = pos;
  return true;
}

void readTouch() {
  if (!TOUCH_ENABLED) return;
  static uint32_t releasedAt = 0;
  float v = touchRead(TOUCH_PIN);

  bool over = touched ? v > touchBaseline * (1.0f + TOUCH_OFF)
                      : v > touchBaseline * (1.0f + TOUCH_ON);
  if (over) {
    releasedAt = 0;
    if (!touched) {
      touched = true;
      dirty = true;
      Serial.printf("[%lu] touch on (%.0f, baseline %.0f)\n", millis(), v, touchBaseline);
    }
  } else {
    // A finger only raises the reading, so drop to any lower reading at once
    // (the sensor reads high for a while after boot); follow upward drift
    // (temperature, humidity) slowly and only while untouched.
    if (v < touchBaseline) touchBaseline = v;
    else if (v < touchBaseline * (1.0f + TOUCH_OFF)) touchBaseline += 0.02f * (v - touchBaseline);
    if (touched) {
      if (!releasedAt) releasedAt = millis();
      if (millis() - releasedAt >= TOUCH_HOLD_MS) {
        touched = false;
        dirty = true;
        Serial.printf("[%lu] touch off (%.0f, baseline %.0f)\n", millis(), v, touchBaseline);
        if (!pickedUp) requestMove();  // let go mid-move: finish the job
      }
    }
  }
}

void calibrateTouch() {
  if (!TOUCH_ENABLED) return;
  for (int i = 0; i < 5; i++) touchRead(TOUCH_PIN);  // first readings after boot run high
  float sum = 0.0f;
  for (int i = 0; i < 8; i++) sum += touchRead(TOUCH_PIN);
  touchBaseline = sum / 8.0f;
  Serial.printf("Touch baseline %.0f\n", touchBaseline);
}

void updateFromPot() {
  float before = potPos;
  if (!readPot()) return;
  Strip &s = strips[selected];
  if (!s.haveFader) return;
  if (!touched) {
    if (!pickedUp) dirty = true;  // keep the position marker current
    return;
  }

  if (!pickedUp) {
    // Take over once the fader reaches or crosses the channel's level
    bool crossed = (before < s.fader) != (potPos < s.fader);
    if (crossed || fabsf(potPos - s.fader) <= PICKUP_RANGE) {
      pickedUp = true;
      Serial.printf("[%lu] picked up %s\n", millis(), s.label);
    } else {
      dirty = true;  // move the position marker
      return;
    }
  }

  if (potPos != s.fader) {
    s.fader = potPos;
    s.sendDue = s.needsFinal = true;
    s.lockedUntil = millis() + LOCK_MS;
    dirty = true;
  }
}

// ---------- OLED ----------

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

void formatDb(char *out, size_t size, float f) {
  if (f <= 0.0f) { snprintf(out, size, "-inf dB"); return; }
  snprintf(out, size, "%+.1f dB", faderToDb(f));
}

void drawStatus(const char *line1, const char *line2) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB10_tr);
  oled.drawStr(0, 24, line1);
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(0, 44, line2);
  oled.sendBuffer();
}

void drawMixer() {
  const Strip &s = strips[selected];
  char text[16];
  oled.clearBuffer();

  // Header: channel number, pickup arrow, level
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(0, 10, s.label);
  if (s.haveFader && !pickedUp) {
    int x = oled.getStrWidth(s.label) + 5;
    if (potPos < s.fader) oled.drawTriangle(x, 9, x + 8, 9, x + 4, 2);  // move up
    else                  oled.drawTriangle(x, 2, x + 8, 2, x + 4, 9);  // move down
  }
  if (touched && TOUCH_ENABLED) oled.drawDisc(oled.getStrWidth(s.label) + 20, 6, 3);
  if (s.haveFader) formatDb(text, sizeof(text), s.fader);
  else strcpy(text, "...");
  oled.drawStr(128 - oled.getStrWidth(text), 10, text);

  // Channel name, shrinking the font if it doesn't fit
  const char *name = s.name[0] ? s.name : s.label;
  oled.setFont(u8g2_font_helvB12_tr);
  if (oled.getStrWidth(name) > 128) oled.setFont(u8g2_font_helvB10_tr);
  if (oled.getStrWidth(name) > 128) oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(0, 28, name);

  // One bar per strip, a dotted line at 0 dB, a marker under the selection
  const int top = 33, bottom = 60, height = bottom - top + 1, pitch = 7, x0 = 1;
  int unityY = bottom - (int)roundf(0.75f * height);
  for (int x = 0; x < 128; x += 2) oled.drawPixel(x, unityY);
  for (int i = 0; i < NUM_STRIPS; i++) {
    int x = x0 + i * pitch;
    if (strips[i].haveFader) {
      int h = (int)roundf(strips[i].fader * height);
      if (h > 0) oled.drawBox(x, bottom - h + 1, 5, h);
    }
    if (i == selected) oled.drawBox(x - 1, 62, 7, 2);
  }

  // Where the slide fader is, while it hasn't picked up the channel yet
  if (s.haveFader && !pickedUp) {
    int y = bottom - (int)roundf(potPos * (height - 1));
    oled.setDrawColor(2);  // XOR so it shows on top of the bar too
    oled.drawHLine(x0 + selected * pitch - 1, y, 7);
    oled.setDrawColor(1);
  }
  oled.sendBuffer();
}

void updateDisplay() {
  static uint32_t lastDraw = 0;
  static int lastMode = -1;
  bool online = netState == NET_CONNECTED && WiFi.status() == WL_CONNECTED;
  int mode = netState == NET_CONNECTING ? 0 : !online ? 1 : !haveMixer ? 2 : 3;
  if (mode != lastMode) dirty = true;
  if (!dirty || millis() - lastDraw < FRAME_MS) return;
  lastDraw = millis();
  lastMode = mode;
  dirty = false;
  if (mode == 0)      drawStatus("Joining", netSsid);
  else if (mode == 1) drawStatus("Wi-Fi", "Pick one on screen");
  else if (mode == 2) drawStatus("Finding XR18", "on the network...");
  else                drawMixer();
}

// ---------- TFT ----------

#include "ui.h"

// ---------- main ----------

void setup() {
  Serial.begin(115200);
  // With USB plugged in but no serial monitor open, each print would block
  // for up to 100 ms waiting for a reader and freeze the display. Drop
  // output instead.
  Serial.setTxTimeoutMs(0);
  stateLock = xSemaphoreCreateMutex();
  uiQueue = xQueueCreate(8, sizeof(UiAction));
  initStrips();

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), onEncoder, CHANGE);
  analogReadResolution(12);
  calibrateTouch();

  uiBegin();  // TFT splash, touchscreen calibration, then the UI task

  oled.setBusClock(400000);
  oled.begin();
  drawStatus("Calibrating", "fader...");
  motorBegin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // lower latency
  WiFi.setAutoReconnect(true);
  xSemaphoreTake(stateLock, portMAX_DELAY);
  loadSavedNetworks();
  autoJoinPending = savedCount > 0;
  startScan();
  xSemaphoreGive(stateLock);
  bootStage = 1;
}

void loop() {
  uint32_t now = millis();
  xSemaphoreTake(stateLock, portMAX_DELAY);
  netService();
  bool online = netState == NET_CONNECTED && WiFi.status() == WL_CONNECTED;

  int size;
  while (online && (size = udp.parsePacket()) > 0) {
    uint8_t buf[512];
    int n = udp.read(buf, sizeof(buf));
    lastRx = now;
    handlePacket(buf, n, udp.remoteIP());
  }

  if (!online) {
    // nothing to talk to until Wi-Fi is up
  } else if (!haveMixer) {
    if (now - lastProbe > 1000) {  // broadcast discovery
      lastProbe = now;
      sendOsc(IPAddress(255, 255, 255, 255), "/xinfo");
    }
  } else {
    if (lastXremote == 0 || now - lastXremote > XREMOTE_MS) {
      lastXremote = now;
      sendOsc(mixerIp, "/xremote");                  // changes from apps/scenes
      sendOscString(mixerIp, "/meters", "/meters/1"); // live input meters
      if (sweepPos < 0) {                            // re-read every strip
        sweepPos = 0;
        sweepRepair = false;
        sweepRepairs = 0;
      }
      if (detailPos < 0) {                           // and the selected one
        detailStrip = selected;
        detailPos = 0;
      }
    }
    runQueries(now);

    if (lastRx && now - lastRx > TIMEOUT_MS) {
      Serial.println("Lost contact with mixer");
      lastRx = 0;
      resetMixerState();
      if (!strlen(XR18_IP)) haveMixer = false;  // rediscover
      else syncStart = millis();
      dirty = true;
    }
  }

  int clicks = takeClicks();
  if (clicks) selectStrip(selected + clicks);

  UiAction action;
  while (xQueueReceive(uiQueue, &action, 0) == pdTRUE) handleUiAction(action);

  static uint32_t lastTouchRead = 0;
  if (now - lastTouchRead >= TOUCH_READ_MS) {
    lastTouchRead = now;
    readTouch();
  }

  if (motorArrived) {
    motorArrived = false;
    pickedUp = true;
    potPos = strips[selected].fader;  // the deadband now tracks from here
    dirty = true;
  }

  static uint32_t lastPotRead = 0;
  if (now - lastPotRead >= POT_READ_MS) {
    lastPotRead = now;
    if (haveMixer) updateFromPot();
    else readPot();
  }

  for (Strip &s : strips) {
    if (!online) break;
    if (s.sendDue && now - lastSend >= SEND_GAP_MS) {
      lastSend = now;
      s.sendDue = false;
      sendStripOsc(s, "/mix/fader", true);
    }
    // Once the fader stops, send the final value again (in case a packet
    // was lost) and ask the mixer to confirm it.
    if (s.needsFinal && !s.sendDue && now >= s.lockedUntil) {
      s.needsFinal = false;
      sendStripOsc(s, "/mix/fader", true);
      sendStripOsc(s, "/mix/fader");
    }
  }

  xSemaphoreGive(stateLock);
  updateDisplay();
  delay(1);
}
