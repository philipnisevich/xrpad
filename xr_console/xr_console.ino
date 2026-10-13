// XR18 console — LCKFB ESP32-S3 core board (N8R8) over Wi-Fi/OSC.
//
// Turn the encoder to pick a channel (CH 1-16, AUX, MAIN). The motorized
// fader drives itself to that channel's level, and follows changes made from
// the app; moving it by hand sets the level. The OLED shows a compact
// readout, and the 4.0" TFT (ui.h) shows everything about the selected
// channel: level and live meter, EQ curve, gate and compressor curves,
// preamp, pan, sends and an overview of all channels. Tap any setting on the
// TFT to point the encoder at it (tap it again to go back to changing
// channels; press the encoder to reset it); EQ, gate, compressor and sends
// are edited on their own pages, opened by tapping their section.
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
//   Fader board (xr_fader_board) G13 -> its RX, G14 <- its TX, GND
//                              (its RX/TX are G46/G45 with an MPR121, else G47/G48)
//   Second fader board (5-8)   G1 -> its RX G47, G2 <- its TX G48, GND
//                              (only for two 4-fader boards without an MPR121)
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
const uint8_t  ENC_SW       = 4;      // encoder push: reset the setting being edited
const uint8_t  POT_PIN      = 7;      // must be ADC1 (G1-G10); ADC2 is unusable with Wi-Fi
const uint8_t  TOUCH_PIN    = 10;     // any of G1-G14 can sense touch
const bool     TOUCH_ENABLED = true;  // false = fader always live (no touch wire)
const uint8_t  OLED_SDA     = 8;
const uint8_t  OLED_SCL     = 9;
const bool     ENC_REVERSE  = false;  // flip if clockwise goes to the previous channel
const bool     POT_REVERSE  = false;  // flip if pushing the fader up lowers the level
const uint8_t  MOTOR_IN1    = 12;
const uint8_t  MOTOR_IN2    = 11;
const uint8_t  FB_TX        = 13;     // link to the fader board (its RX, G47)
const uint8_t  FB_RX        = 14;     // (its TX, G48)
const uint8_t  FB2_TX       = 1;      // link to a second fader board for faders 5-8
const uint8_t  FB2_RX       = 2;      // (its RX G47, its TX G48)
const int      FB2_FIRST    = 4;      // bank fader the second board's fader 1 is (0-based)
const uint32_t FB_BAUD      = 921600;
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
// Measured (USB "V"): speed = MOTOR_K * (power - MOTOR_D0) travel/s.
const float    MOTOR_K      = 17.0f;
const float    MOTOR_D0     = 0.22f;  // power that just balances friction
const float    MOTOR_KP     = 6.0f;   // power per unit of lag behind the glide
const float    MOTOR_KD     = 0.08f;  // power per travel/s faster than the glide (damping)
const float    MOTOR_FULL_FRICTION_VEL = 0.5f; // glide speed (travel/s) from which friction is fully offset
const float    MOVE_VMAX    = 6.0f;   // glide speed limit, travel/s
const float    MOVE_AMAX    = 60.0f;  // glide acceleration, travel/s^2
const float    MOTOR_TOL    = 0.008f; // close enough to stop
const float    MOTOR_STILL_VEL = 0.3f; // travel/s: slower than this counts as stopped
const uint32_t MOTOR_HOLD_MS = 80;    // brake this long at the end of a move
const uint32_t MOTOR_SETTLE_CHECK_MS = 80; // after that, check it stayed put
const uint8_t  MOTOR_CORRECTIONS = 3; // gentle nudges back if it drifted off
const uint32_t MOTOR_STALL_MS = 25;   // not moving this long while it should: push harder
const uint32_t MOTOR_KICK_MS = 150;   // ...ramping to full power over this long
const float    MOTOR_HELD_LAG = 0.2f; // this far behind the glide...
const uint32_t MOTOR_HELD_MS = 250;   // ...for this long = held or blocked: let go
const uint32_t MOTOR_TIMEOUT_MS = 800; // glide finished but still not there: give up
const uint32_t MOTOR_FOLLOW_MS = 300; // targets closer together than this = a stream

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
  uint32_t lastSendAt;
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
  float pan, gain, hpf;
  float gateThr, gateRange, gateAtt, gateHold, gateRel;
  float dynThr, dynKnee, dynAtt, dynRel, dynMgain, dynMix;
  EqBand eq[4];
  float sends[10];   // bus 1-6, FX 1-4
  int8_t insrc, fxslot, insOn;  // input source, insert FX slot (0 off, 1-8 FX1A..FX4B), insert on
};

// Everything that isn't a channel strip: FX returns 1-4, buses 1-6, FX sends
// 1-4 and DCAs 1-4, shown on the home screen.
const int NUM_OTHERS = 18, OTHER_RTN = 0, OTHER_BUS = 4, OTHER_FXSEND = 10, OTHER_DCA = 14;
const int OTHER_PHONES = 18;  // P_OTHER index for the headphone level
struct Other { char prefix[12]; char label[8]; char name[16]; float fader; int8_t on, color; };
Other others[NUM_OTHERS];

// Mixer-wide settings for the home screen (-1 = not reported yet).
struct MixerInfo {
  int8_t phonesSrc;
  int8_t muteGrp[4], chLink[8], busLink[3], amix[2], fxType[4];
  float phonesLevel;
  int16_t snapIndex;
  char snapName[24];
} mixInfo;

// Everything the encoder can edit. Index (idx) picks the EQ band or send.
enum ParamId : uint8_t {
  P_NONE, P_FADER, P_PAN, P_GAIN, P_PHANTOM, P_INVERT, P_LR, P_HPON, P_HPF,
  P_EQON, P_EQ_TYPE, P_EQ_F, P_EQ_G, P_EQ_Q,
  P_GATE_ON, P_GATE_THR, P_GATE_RANGE, P_GATE_ATT, P_GATE_HOLD, P_GATE_REL,
  P_DYN_ON, P_DYN_THR, P_DYN_RATIO, P_DYN_KNEE, P_DYN_ATT, P_DYN_REL, P_DYN_MGAIN, P_DYN_MIX,
  P_SEND, P_OTHER, P_INS_SLOT, P_INS_ON, P_COUNT
};

// kind: 'f' 0-1 float in `steps` steps, 'i' int 0..steps, 'b' on/off,
// 'F' fader-law level (stepped in dB). %d in the suffix is the band/send.
struct ParamInfo { const char *suffix; char kind; int steps; const char *name; };
const ParamInfo PARAMS[P_COUNT] = {
  {"", 0, 0, ""},
  {"/mix/fader", 'F', 0, "FADER"},
  {"/mix/pan", 'f', 100, "PAN"},
  {"gain", 'f', 144, "GAIN"},          // /headamp/NN/gain
  {"phantom", 'b', 1, "48V"},          // /headamp/NN/phantom
  {"/preamp/invert", 'b', 1, "INVERT"},
  {"/mix/lr", 'b', 1, "LR"},
  {"/preamp/hpon", 'b', 1, "LOW CUT"},
  {"/preamp/hpf", 'f', 100, "LOW CUT FREQ"},
  {"/eq/on", 'b', 1, "EQ"},
  {"/eq/%d/type", 'i', 5, "TYPE"},
  {"/eq/%d/f", 'f', 200, "FREQ"},
  {"/eq/%d/g", 'f', 120, "GAIN"},
  {"/eq/%d/q", 'f', 71, "Q"},
  {"/gate/on", 'b', 1, "GATE"},
  {"/gate/thr", 'f', 160, "THRESHOLD"},
  {"/gate/range", 'f', 57, "RANGE"},
  {"/gate/attack", 'f', 120, "ATTACK"},
  {"/gate/hold", 'f', 100, "HOLD"},
  {"/gate/release", 'f', 100, "RELEASE"},
  {"/dyn/on", 'b', 1, "COMP"},
  {"/dyn/thr", 'f', 120, "THRESHOLD"},
  {"/dyn/ratio", 'i', 11, "RATIO"},
  {"/dyn/knee", 'f', 5, "KNEE"},
  {"/dyn/attack", 'f', 120, "ATTACK"},
  {"/dyn/release", 'f', 100, "RELEASE"},
  {"/dyn/mgain", 'f', 48, "MAKEUP"},
  {"/dyn/mix", 'f', 20, "MIX"},
  {"/mix/%02d/level", 'F', 0, "SEND"},
  {"", 'F', 0, "LEVEL"},               // others[idx] fader, or headphones
  {"/insert/sel", 'i', 8, "INSERT"},  // the X-Air docs say insert/fxslot; the mixer answers sel
  {"/insert/on", 'b', 1, "INSERT"},
};

// What the encoder edits; P_NONE = it changes channels.
volatile uint8_t editId = P_NONE, editIdx = 0;

Detail details[NUM_STRIPS];

// Live input meters (dBFS).
float meterDb[NUM_STRIPS];
float allMeters[40];  // the whole /meters/1 block, for the home meter bridge
uint32_t meterSeq = 0;

void resetDetail(Detail &d) {
  d.color = d.lr = d.phantom = d.hpOn = d.invert = d.gateOn = d.dynOn = -1;
  d.dynRatio = d.eqOn = d.solo = -1;
  d.pan = d.gain = d.hpf = NAN;
  d.gateThr = d.gateRange = d.gateAtt = d.gateHold = d.gateRel = NAN;
  d.dynThr = d.dynKnee = d.dynAtt = d.dynRel = d.dynMgain = d.dynMix = NAN;
  d.insrc = d.fxslot = d.insOn = -1;
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
  for (int i = 0; i < NUM_OTHERS; i++) {
    Other &o = others[i];
    if (i < OTHER_BUS)         { snprintf(o.prefix, 12, "/rtn/%d", i + 1);    snprintf(o.label, 8, "FX %d", i + 1); }
    else if (i < OTHER_FXSEND) { snprintf(o.prefix, 12, "/bus/%d", i - 3);    snprintf(o.label, 8, "BUS %d", i - 3); }
    else if (i < OTHER_DCA)    { snprintf(o.prefix, 12, "/fxsend/%d", i - 9); snprintf(o.label, 8, "SND %d", i - 9); }
    else                       { snprintf(o.prefix, 12, "/dca/%d", i - 13);   snprintf(o.label, 8, "DCA %d", i - 13); }
    o.name[0] = 0;
    o.fader = NAN;
    o.on = o.color = -1;
  }
  memset(&mixInfo, -1, sizeof(mixInfo));
  mixInfo.phonesLevel = NAN;
  mixInfo.snapName[0] = 0;
  for (float &m : allMeters) m = NAN;
  for (int i = 0; i < NUM_STRIPS; i++) {
    strips[i].on = -1;
    resetDetail(details[i]);
    meterDb[i] = NAN;
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
enum UiActionType : uint8_t { UI_SELECT, UI_MUTE, UI_SOLO, UI_SCAN, UI_EDIT, UI_TOGGLE,
                              UI_MUTE_ANY, UI_MUTEGRP, UI_SET, UI_BANK };
struct UiAction { UiActionType type; int8_t arg; uint8_t arg2; };
QueueHandle_t uiQueue;
void handleUiAction(const UiAction &a);
extern char editLockAddr[40];
extern uint32_t editLockUntil;
void toggleParam(uint8_t id, uint8_t idx);
void editParam(int clicks, bool fast);

// ---------- network state ----------

WiFiUDP udp;
IPAddress mixerIp;
volatile bool haveMixer = false;
uint32_t lastRx = 0, lastXremote = 0, lastProbe = 0, lastQuery = 0;
int sweepPos = -1;   // position in the all-strips query sweep, -1 = idle
int extraPos = -1;   // position in the mixer-wide (home screen) queries
uint32_t lastExtra = 0;
const uint32_t EXTRA_EVERY_MS = 60000;
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
volatile bool motorMoving = false;

// One input is held high and the other gets the inverted PWM, so the motor
// alternates between drive and brake (slow decay). Its speed then follows the
// power evenly, both ways; drive / coast needed far more power to move at all
// and then lurched. 0 = coast.
void motorDrive(float speed) {  // -1.0 (down) .. 1.0 (up)
  if (MOTOR_REVERSE) speed = -speed;
  const uint32_t full = (1 << MOTOR_PWM_BITS) - 1;  // the core treats this as fully on
  uint32_t off = (uint32_t)((1.0f - min(fabsf(speed), 1.0f)) * full);
  if (speed == 0.0f) {
    ledcWrite(MOTOR_IN1, 0);
    ledcWrite(MOTOR_IN2, 0);
  } else if (speed > 0.0f) {
    ledcWrite(MOTOR_IN1, full);
    ledcWrite(MOTOR_IN2, off);
  } else {
    ledcWrite(MOTOR_IN2, full);
    ledcWrite(MOTOR_IN1, off);
  }
}

void motorBrake() {  // both inputs high shorts the motor
  ledcWrite(MOTOR_IN1, (1 << MOTOR_PWM_BITS) - 1);
  ledcWrite(MOTOR_IN2, (1 << MOTOR_PWM_BITS) - 1);
}

float readFaderPos() {
  uint32_t mv = 0;
  for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(POT_PIN);
  mv /= 4;
  float raw = constrain(((float)mv - potMinMv) / (potMaxMv - potMinMv), 0.0f, 1.0f);
  return POT_REVERSE ? 1.0f - raw : raw;
}

// DEBUG over USB serial: "V" sweeps motor power against speed; "G <0-1000>"
// moves to a position; "S <from> <to> <ms> <step ms>" sends a stream of
// targets like the app or encoder; "R" prints the last recording (time,
// glide setpoint, position, power, every 2 ms).
volatile bool sweepRequest = false;
const int REC_LEN = 1000;
struct RecSample { uint16_t t; int16_t sp, pos, power; };
RecSample rec[REC_LEN];
volatile int recCount = REC_LEN;
uint32_t recStart = 0;

void sweepMotor() {
  for (int dir = 1; dir >= -1; dir -= 2) {
    for (int pct = 10; pct <= 60; pct += 5) {
      motorDrive(-dir * 1.0f);  // back to the start end
      delay(350);
      motorDrive(0.0f);
      delay(80);
      uint32_t t0 = millis(), tA = 0, tB = 0;
      float start = readFaderPos();
      motorDrive(dir * pct / 100.0f);
      while (millis() - t0 < 1500) {
        float q = dir > 0 ? readFaderPos() : 1.0f - readFaderPos();
        if (!tA && q >= 0.3f) tA = millis();
        if (!tB && q >= 0.7f) { tB = millis(); break; }
        delay(1);
      }
      motorDrive(0.0f);
      Serial.printf("sweep %s %3d%%: ", dir > 0 ? "up  " : "down", pct);
      if (tA && tB) Serial.printf("%.2f travel/s\n", 0.4f / ((tB - tA) / 1000.0f));
      else Serial.printf("slow, moved %.3f in 1.5 s\n", fabsf(readFaderPos() - start));
      delay(50);
    }
  }
}

void printRecording() {
  Serial.printf("rec %d samples\n", recCount);
  for (int i = 0; i < recCount; i++) Serial.printf("r %u %d %d %d\n", rec[i].t, rec[i].sp, rec[i].pos, rec[i].power);
  Serial.println("rec end");
}

// Moves follow a glide: a setpoint that speeds up and slows down smoothly
// towards the target, which the motor tracks. A stream of targets (the app's
// fader, the encoder) arrives in small steps; the glide plays them back as
// one steady motion instead of stopping at each step.
void motorTask(void *) {
  float smooth = -1.0f, vel = 0.0f;
  float target = -1.0f, sp = 0.0f, spVel = 0.0f;
  float streamVel = 0.0f, progressPos = 0.0f, power = 0.0f;
  uint32_t streamGap = 0, lastTargetAt = 0, arrivedAt = 0, lagSince = 0;
  uint32_t progressAt = 0, brakeUntil = 0, checkAt = 0, seenSeq = 0;
  uint8_t corrections = 0;
  bool moving = false;
  uint32_t lastUs = micros(), lastRec = 0;

  auto startGlide = [&](uint32_t now) {
    moving = true;
    sp = smooth;
    spVel = 0;
    arrivedAt = lagSince = 0;
    progressAt = now;
    progressPos = smooth;
  };
  auto stop = [&]() {
    motorDrive(0.0f);
    power = 0;
    moving = false;
    spVel = 0;
  };

  for (;;) {
    uint32_t us = micros();
    float dt = min((us - lastUs) / 1e6f, 0.01f);
    lastUs = us;
    uint32_t now = millis();

    if (sweepRequest) {
      stop();
      sweepMotor();
      sweepRequest = false;
    }

    float prev = smooth;
    float pos = readFaderPos();
    smooth = smooth < 0.0f ? pos : smooth + 0.3f * (pos - smooth);
    if (prev >= 0.0f && dt > 0.0f) vel += 0.1f * ((smooth - prev) / dt - vel);
    potSmooth = smooth;

    if (motorSeq != seenSeq) {  // new target
      seenSeq = motorSeq;
      float t = motorTarget;
      uint32_t gap = now - lastTargetAt;
      bool stream = moving && gap < MOTOR_FOLLOW_MS;  // slow turns: a quick step each
      lastTargetAt = now;
      target = t;
      checkAt = 0;
      corrections = 0;
      arrivedAt = 0;
      if (!touched) {
        if (!moving) startGlide(now);
        // Aim to reach each value 1.5 steps from now, so the glide is still
        // on its way when the next one arrives instead of stopping dead.
        if (stream) {
          streamGap = constrain(gap, (uint32_t)10, (uint32_t)100);
          streamVel = constrain((target - sp) * 1000.0f / (streamGap * 1.5f), -MOVE_VMAX, MOVE_VMAX);
        } else {
          streamGap = 0;
          streamVel = 0;
        }
      }
    }

    if (brakeUntil && (now >= brakeUntil || touched || moving)) {
      if (!moving) motorDrive(0.0f);
      brakeUntil = 0;
    }
    // After stopping, check it stayed on target; nudge it back gently if not.
    if (!moving && checkAt && now >= checkAt) {
      checkAt = 0;
      if (!touched && fabsf(target - smooth) > MOTOR_TOL * 2 && corrections < MOTOR_CORRECTIONS) {
        corrections++;
        startGlide(now);
      }
    }

    if (moving && touched) {
      stop();
      Serial.printf("[%lu] motor: hand on fader, letting go\n", now);
    } else if (moving) {
      // Advance the glide: the fastest speed that still slows down smoothly
      // onto the target; a stream also keeps to its own speed.
      if (streamGap && now - lastTargetAt > streamGap * 3 / 2 + 20) { streamVel = 0; streamGap = 0; }
      float e = target - sp;
      float want = copysignf(sqrtf(2.0f * MOVE_AMAX * fabsf(e)), e);
      if (streamGap) want = streamVel * want >= 0.0f ? copysignf(min(fabsf(streamVel), fabsf(want)), want) : 0.0f;
      want = constrain(want, -MOVE_VMAX, MOVE_VMAX);
      spVel += constrain(want - spVel, -MOVE_AMAX * dt, MOVE_AMAX * dt);
      float next = sp + spVel * dt;
      if ((next - target) * (sp - target) <= 0.0f) { next = target; spVel = 0; }  // never past it
      sp = constrain(next, 0.0f, 1.0f);

      float err = target - smooth;
      float lag = sp - smooth;
      bool arrived = sp == target && !streamGap;
      if (arrived && !arrivedAt) arrivedAt = now;

      if (arrived && fabsf(err) <= MOTOR_TOL && fabsf(vel) < MOTOR_STILL_VEL) {
        // Hold it with the brake for a moment, or it slides on a little.
        stop();
        motorBrake();
        brakeUntil = now + MOTOR_HOLD_MS;
        checkAt = brakeUntil + MOTOR_SETTLE_CHECK_MS;
        motorArrived = true;
      } else if (arrived && now - arrivedAt > MOTOR_TIMEOUT_MS) {
        stop();
        motorArrived = true;
        Serial.printf("[%lu] motor: gave up, %.3f from target\n", now, err);
      } else if (fabsf(lag) > MOTOR_HELD_LAG && lagSince && now - lagSince > MOTOR_HELD_MS) {
        stop();
        Serial.printf("[%lu] motor: held or blocked, letting go\n", now);
      } else {
        if (fabsf(lag) > MOTOR_HELD_LAG) { if (!lagSince) lagSince = now; }
        else lagSince = 0;
        // Power: what the glide's speed needs (friction plus speed), plus a
        // correction for lagging behind it, and damping if it runs faster
        // than the glide. Friction is only added in the glide's direction,
        // and in full only at speed: flipping it with every tiny error made
        // the motor chatter. Stuck behind: push harder, gradually.
        float u = MOTOR_KP * lag + MOTOR_KD * (spVel - vel);
        if (fabsf(spVel) > 0.01f)
          u += copysignf(MOTOR_D0 * min(1.0f, fabsf(spVel) / MOTOR_FULL_FRICTION_VEL) + fabsf(spVel) / MOTOR_K, spVel);
        else if (fabsf(lag) <= MOTOR_TOL)
          u = 0;
        if (fabsf(smooth - progressPos) > 0.004f || fabsf(lag) <= MOTOR_TOL) {
          progressPos = smooth;
          progressAt = now;
        }
        uint32_t stuck = now - progressAt;
        if (stuck > MOTOR_STALL_MS) u += copysignf((float)(stuck - MOTOR_STALL_MS) / MOTOR_KICK_MS, lag);
        power = constrain(u, -1.0f, 1.0f);
        motorDrive(power);
      }
    }
    motorMoving = moving;

    if (recCount < REC_LEN && now - lastRec >= 2) {
      lastRec = now;
      rec[recCount] = {(uint16_t)(now - recStart), (int16_t)lroundf((moving ? sp : target) * 1000),
                       (int16_t)lroundf(smooth * 1000), (int16_t)lroundf(power * 1000)};
      recCount = recCount + 1;
    }
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

void setMotorTarget(float t) {
  motorTarget = t;
  motorSeq = motorSeq + 1;
}

// DEBUG motor commands over USB serial (see sweepMotor above).
void debugCommands(uint32_t now) {
  static char cmd[40];
  static int len = 0;
  static int sFrom = 0, sTo = 0;
  static uint32_t sMs = 0, sStep = 0, sStart = 0, sLast = 0;
  static bool streaming = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < (int)sizeof(cmd) - 1) cmd[len++] = c;
      continue;
    }
    cmd[len] = 0;
    int a, b, ms, step;
    if (len && cmd[0] == 'V') sweepRequest = true;
    if (len && cmd[0] == 'R') printRecording();
    if (len && cmd[0] == 'G' && sscanf(cmd + 1, "%d", &a) == 1) {
      recStart = now;
      recCount = 0;
      setMotorTarget(constrain(a, 0, 1000) / 1000.0f);
    }
    if (len && cmd[0] == 'S' && sscanf(cmd + 1, "%d %d %d %d", &a, &b, &ms, &step) == 4 && ms > 0 && step > 0) {
      recStart = now;
      recCount = 0;
      sFrom = a; sTo = b; sMs = ms; sStep = step; sStart = sLast = now;
      streaming = true;
      setMotorTarget(constrain(a, 0, 1000) / 1000.0f);
    }
    len = 0;
  }
  if (streaming && now - sLast >= sStep) {
    sLast = now;
    float k = min(1.0f, (float)(now - sStart) / sMs);
    setMotorTarget(constrain(sFrom + k * (sTo - sFrom), 0.0f, 1000.0f) / 1000.0f);
    if (k >= 1.0f) streaming = false;
  }
}

// Drive the fader to the selected channel's level.
void requestMove() {
  const Strip &s = strips[selected];
  if (!s.haveFader || potSmooth < 0.0f) return;
  if (!motorMoving && fabsf(potSmooth - s.fader) <= MOTOR_TOL) {
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
  for (Other &o : others) { o.name[0] = 0; o.fader = NAN; o.on = o.color = -1; }
  memset(&mixInfo, -1, sizeof(mixInfo));
  mixInfo.phonesLevel = NAN;
  mixInfo.snapName[0] = 0;
  for (float &m : allMeters) m = NAN;
  extraPos = -1;
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
  for (int i = 0; i < 40 && i < (int)count; i++) allMeters[i] = level(i);
  meterDb[AUX_STRIP] = max(level(16), level(17));
  meterDb[MAIN_STRIP] = max(level(36), level(37));
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
  else if (!strcmp(suffix, "/gate/range"))    d.gateRange = v;
  else if (!strcmp(suffix, "/gate/attack"))   d.gateAtt = v;
  else if (!strcmp(suffix, "/gate/hold"))     d.gateHold = v;
  else if (!strcmp(suffix, "/gate/release"))  d.gateRel = v;
  else if (!strcmp(suffix, "/dyn/on"))        d.dynOn = iv;
  else if (!strcmp(suffix, "/dyn/thr"))       d.dynThr = v;
  else if (!strcmp(suffix, "/dyn/ratio"))     d.dynRatio = iv;
  else if (!strcmp(suffix, "/dyn/knee"))      d.dynKnee = v;
  else if (!strcmp(suffix, "/dyn/attack"))    d.dynAtt = v;
  else if (!strcmp(suffix, "/dyn/release"))   d.dynRel = v;
  else if (!strcmp(suffix, "/dyn/mgain"))     d.dynMgain = v;
  else if (!strcmp(suffix, "/dyn/mix"))       d.dynMix = v;
  else if (!strcmp(suffix, "/eq/on"))         d.eqOn = iv;
  else if (!strcmp(suffix, "/config/insrc"))  d.insrc = iv;
  else if (!strcmp(suffix, "/insert/sel"))    d.fxslot = iv;
  else if (!strcmp(suffix, "/insert/on"))     d.insOn = iv;
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

// Index into others[] for /rtn/N, /bus/N, /fxsend/N, /dca/N (or -1), and
// where the rest of the address starts.
int otherFor(const char *address, const char **suffix) {
  struct { const char *p; int base, count; } kinds[] = {
    {"/rtn/", OTHER_RTN, 4}, {"/bus/", OTHER_BUS, 6}, {"/fxsend/", OTHER_FXSEND, 4}, {"/dca/", OTHER_DCA, 4},
  };
  for (auto &k : kinds) {
    size_t n = strlen(k.p);
    if (strncmp(address, k.p, n) || !isdigit((unsigned char)address[n])) continue;
    int i = address[n] - '1';
    if (i < 0 || i >= k.count || address[n + 1] != '/') return -1;
    *suffix = address + n + 1;
    return k.base + i;
  }
  return -1;
}

// Routing, buses, DCAs, FX, monitor and snapshot replies. True if handled.
bool handleMixerWide(const char *address, const char *tags, const uint8_t *arg, size_t argLen) {
  float v;
  const char *suffix;
  int oi = otherFor(address, &suffix);
  if (oi >= 0) {
    Other &o = others[oi];
    if (!strcmp(suffix, "/config/name") && !strcmp(tags, ",s")) {
      strlcpy(o.name, (const char *)arg, min(argLen + 1, sizeof(o.name)));
    } else if (readNumber(tags, arg, argLen, v)) {
      if (!strcmp(suffix, "/mix/fader") || !strcmp(suffix, "/fader")) o.fader = v;
      else if (!strcmp(suffix, "/mix/on") || !strcmp(suffix, "/on")) o.on = (int8_t)v;
      else if (!strcmp(suffix, "/config/color")) o.color = (int8_t)v;
    }
    return true;
  }
  MixerInfo &m = mixInfo;
  if (!strncmp(address, "/-snap/", 7)) {
    if (!strcmp(address + 7, "name") && !strcmp(tags, ",s"))
      strlcpy(m.snapName, (const char *)arg, min(argLen + 1, sizeof(m.snapName)));
    else if (!strcmp(address + 7, "index") && readNumber(tags, arg, argLen, v)) m.snapIndex = (int16_t)v;
    return true;
  }
  if (!readNumber(tags, arg, argLen, v)) return false;
  int8_t iv = (int8_t)v;
  int n;
  if (!strcmp(address, "/config/solo/level")) m.phonesLevel = v;
  else if (!strcmp(address, "/config/solo/source")) m.phonesSrc = iv;
  else if (sscanf(address, "/config/mute/%d", &n) == 1 && n >= 1 && n <= 4) m.muteGrp[n - 1] = iv;
  else if (sscanf(address, "/config/chlink/%d", &n) == 1 && n >= 1 && n <= 15) m.chLink[n / 2] = iv;
  else if (sscanf(address, "/config/buslink/%d", &n) == 1 && n >= 1 && n <= 5) m.busLink[n / 2] = iv;
  else if (!strcmp(address, "/config/amixenable/X")) m.amix[0] = iv;
  else if (!strcmp(address, "/config/amixenable/Y")) m.amix[1] = iv;
  else if (sscanf(address, "/fx/%d/type", &n) == 1 && n >= 1 && n <= 4) m.fxType[n - 1] = iv;
  else return false;
  return true;
}

// Everything the home screen shows that the per-channel queries don't
// cover. Returns false past the end.
bool extraQuery(int idx, char *a, size_t n) {
  static const char *const CH[] = {"/config/color", "/mix/pan", "/mix/lr", "/eq/on", "/gate/on",
                                   "/dyn/on", "/config/insrc", "/insert/sel", "/insert/on"};
  const int nCh = 9;
  if (idx < 16 * nCh) { snprintf(a, n, "/ch/%02d%s", idx / nCh + 1, CH[idx % nCh]); return true; }
  idx -= 16 * nCh;
  static const char *const AUXLR[] = {"/rtn/aux/config/color", "/rtn/aux/mix/lr", "/rtn/aux/eq/on", "/lr/config/color"};
  if (idx < 4) { snprintf(a, n, "%s", AUXLR[idx]); return true; }
  idx -= 4;
  if (idx < 16) { snprintf(a, n, "/headamp/%02d/phantom", idx + 1); return true; }
  idx -= 16;
  if (idx < 17) { snprintf(a, n, "/-stat/solosw/%02d", idx + 1); return true; }
  idx -= 17;
  if (idx < NUM_OTHERS * 4) {
    const Other &o = others[idx / 4];
    bool dca = idx / 4 >= OTHER_DCA;
    static const char *const MIX[] = {"/config/name", "/mix/fader", "/mix/on", "/config/color"};
    static const char *const DCA[] = {"/config/name", "/fader", "/on", "/config/color"};
    snprintf(a, n, "%s%s", o.prefix, (dca ? DCA : MIX)[idx % 4]);
    return true;
  }
  idx -= NUM_OTHERS * 4;
  if (idx < 8) { snprintf(a, n, "/config/chlink/%d-%d", idx * 2 + 1, idx * 2 + 2); return true; }
  idx -= 8;
  if (idx < 3) { snprintf(a, n, "/config/buslink/%d-%d", idx * 2 + 1, idx * 2 + 2); return true; }
  idx -= 3;
  if (idx < 4) { snprintf(a, n, "/config/mute/%d", idx + 1); return true; }
  idx -= 4;
  if (idx < 4) { snprintf(a, n, "/fx/%d/type", idx + 1); return true; }
  idx -= 4;
  static const char *const MISC[] = {"/config/solo/level", "/config/solo/source", "/config/amixenable/X",
                                     "/config/amixenable/Y", "/-snap/index", "/-snap/name"};
  if (idx < 6) { snprintf(a, n, "%s", MISC[idx]); return true; }
  return false;
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

  // While the encoder is changing something, ignore the mixer's (possibly
  // older) copies of it so the value doesn't jump back.
  if (millis() < editLockUntil && !strcmp(address, editLockAddr)) return;

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
  if (handleMixerWide(address, tags, arg, argLen)) return;

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
    "/preamp/invert", "/gate/on", "/gate/thr", "/gate/range", "/gate/attack", "/gate/hold",
    "/gate/release", "/dyn/on", "/dyn/thr", "/dyn/ratio", "/dyn/knee", "/dyn/attack",
    "/dyn/release", "/dyn/mgain", "/dyn/mix", "/eq/on", "/insert/sel", "/insert/on",
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
    return;
  }
  // Last, the home screen's mixer-wide data.
  if (extraPos >= 0 && now - lastQuery >= QUERY_GAP_MS) {
    char addr[40];
    if (extraQuery(extraPos++, addr, sizeof(addr))) {
      lastQuery = now;
      sendOsc(mixerIp, addr);
    } else {
      extraPos = -1;
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
volatile uint32_t encLastClickAt = 0, encClickGap = UINT32_MAX;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEncoder() {
  static const int8_t TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  uint8_t ab = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  portENTER_CRITICAL_ISR(&encMux);
  encState = ((encState << 2) | ab) & 0x0F;
  encEdges += TABLE[encState];
  if (ab == 0b11) {  // back at rest
    if (encEdges >= 2 || encEdges <= -2) {
      encClicks += encEdges > 0 ? 1 : -1;
      uint32_t t = millis();
      encClickGap = t - encLastClickAt;
      encLastClickAt = t;
    }
    encEdges = 0;
  }
  portEXIT_CRITICAL_ISR(&encMux);
}

int takeClicks(uint32_t &gap) {
  portENTER_CRITICAL(&encMux);
  int clicks = encClicks;
  encClicks = 0;
  gap = encClickGap;
  portEXIT_CRITICAL(&encMux);
  return ENC_REVERSE ? -clicks : clicks;
}

bool encButtonPressed() {
  static bool stable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  bool r = digitalRead(ENC_SW);
  if (r != lastRead) { lastRead = r; changedAt = millis(); }
  if (millis() - changedAt > 30 && r != stable) {
    stable = r;
    return stable == LOW;
  }
  return false;
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

// ---------- parameter editing ----------

char editLockAddr[40] = "";
uint32_t editLockUntil = 0;

int inputNumber(int strip) { return strip == AUX_STRIP ? 17 : strip < 16 ? strip + 1 : 0; }

bool paramAddress(int strip, uint8_t id, uint8_t idx, char *addr, size_t size) {
  const ParamInfo &p = PARAMS[id];
  if (id == P_OTHER) {
    if (idx == OTHER_PHONES) snprintf(addr, size, "/config/solo/level");
    else if (idx < NUM_OTHERS)
      snprintf(addr, size, "%s%s", others[idx].prefix, idx >= OTHER_DCA ? "/fader" : "/mix/fader");
    else return false;
    return true;
  }
  if (id == P_GAIN || id == P_PHANTOM) {
    int in = inputNumber(strip);
    if (!in) return false;
    snprintf(addr, size, "/headamp/%02d/%s", in, p.suffix);
    return true;
  }
  char suffix[24];
  snprintf(suffix, sizeof(suffix), p.suffix, idx + 1);
  snprintf(addr, size, "%s%s", strips[strip].prefix, suffix);
  return true;
}

// The stored value of a parameter (NAN if the mixer hasn't reported it).
float paramValue(int strip, uint8_t id, uint8_t idx) {
  const Detail &d = details[strip];
  auto i = [](int8_t v) { return v < 0 ? NAN : (float)v; };
  switch (id) {
    case P_FADER:      return strips[strip].haveFader ? strips[strip].fader : NAN;
    case P_PAN:        return d.pan;
    case P_GAIN:       return d.gain;
    case P_PHANTOM:    return i(d.phantom);
    case P_INVERT:     return i(d.invert);
    case P_LR:         return i(d.lr);
    case P_HPON:       return i(d.hpOn);
    case P_HPF:        return d.hpf;
    case P_EQON:       return i(d.eqOn);
    case P_INS_SLOT:   return i(d.fxslot);
    case P_INS_ON:     return i(d.insOn);
    case P_EQ_TYPE:    return i(d.eq[idx & 3].type);
    case P_EQ_F:       return d.eq[idx & 3].f;
    case P_EQ_G:       return d.eq[idx & 3].g;
    case P_EQ_Q:       return d.eq[idx & 3].q;
    case P_GATE_ON:    return i(d.gateOn);
    case P_GATE_THR:   return d.gateThr;
    case P_GATE_RANGE: return d.gateRange;
    case P_GATE_ATT:   return d.gateAtt;
    case P_GATE_HOLD:  return d.gateHold;
    case P_GATE_REL:   return d.gateRel;
    case P_DYN_ON:     return i(d.dynOn);
    case P_DYN_THR:    return d.dynThr;
    case P_DYN_RATIO:  return i(d.dynRatio);
    case P_DYN_KNEE:   return d.dynKnee;
    case P_DYN_ATT:    return d.dynAtt;
    case P_DYN_REL:    return d.dynRel;
    case P_DYN_MGAIN:  return d.dynMgain;
    case P_DYN_MIX:    return d.dynMix;
    case P_SEND:       return d.sends[idx % 10];
    case P_OTHER:      return idx == OTHER_PHONES ? mixInfo.phonesLevel
                            : idx < NUM_OTHERS ? others[idx].fader : NAN;
  }
  return NAN;
}

void storeParam(int strip, uint8_t id, uint8_t idx, float v) {
  Detail &d = details[strip];
  int8_t iv = (int8_t)lroundf(v);
  switch (id) {
    case P_PAN:        d.pan = v; break;
    case P_GAIN:       d.gain = v; break;
    case P_PHANTOM:    d.phantom = iv; break;
    case P_INVERT:     d.invert = iv; break;
    case P_LR:         d.lr = iv; break;
    case P_HPON:       d.hpOn = iv; break;
    case P_HPF:        d.hpf = v; break;
    case P_EQON:       d.eqOn = iv; break;
    case P_INS_SLOT:   d.fxslot = iv; break;
    case P_INS_ON:     d.insOn = iv; break;
    case P_EQ_TYPE:    d.eq[idx & 3].type = iv; break;
    case P_EQ_F:       d.eq[idx & 3].f = v; break;
    case P_EQ_G:       d.eq[idx & 3].g = v; break;
    case P_EQ_Q:       d.eq[idx & 3].q = v; break;
    case P_GATE_ON:    d.gateOn = iv; break;
    case P_GATE_THR:   d.gateThr = v; break;
    case P_GATE_RANGE: d.gateRange = v; break;
    case P_GATE_ATT:   d.gateAtt = v; break;
    case P_GATE_HOLD:  d.gateHold = v; break;
    case P_GATE_REL:   d.gateRel = v; break;
    case P_DYN_ON:     d.dynOn = iv; break;
    case P_DYN_THR:    d.dynThr = v; break;
    case P_DYN_RATIO:  d.dynRatio = iv; break;
    case P_DYN_KNEE:   d.dynKnee = v; break;
    case P_DYN_ATT:    d.dynAtt = v; break;
    case P_DYN_REL:    d.dynRel = v; break;
    case P_DYN_MGAIN:  d.dynMgain = v; break;
    case P_DYN_MIX:    d.dynMix = v; break;
    case P_SEND:       d.sends[idx % 10] = v; break;
    case P_OTHER:
      if (idx == OTHER_PHONES) mixInfo.phonesLevel = v;
      else if (idx < NUM_OTHERS) others[idx].fader = v;
      break;
  }
}

// Store locally, send to the mixer, and ignore its echoes for a moment.
void setParam(int strip, uint8_t id, uint8_t idx, float v) {
  char addr[40];
  if (!paramAddress(strip, id, idx, addr, sizeof(addr))) return;
  storeParam(strip, id, idx, v);
  char kind = PARAMS[id].kind;
  if (kind == 'f' || kind == 'F') {
    uint32_t raw;
    memcpy(&raw, &v, 4);
    sendOsc(mixerIp, addr, 'f', raw);
  } else {
    sendOsc(mixerIp, addr, 'i', (uint32_t)(int32_t)lroundf(v));
  }
  strlcpy(editLockAddr, addr, sizeof(editLockAddr));
  editLockUntil = millis() + 500;
}

// Fader-law levels step in dB, like a real fader: 0.5 dB, 2 dB when fast.
float stepFaderLaw(float f, int clicks, bool fast) {
  float step = fast ? 2.0f : 0.5f;
  float db = f <= 0.0f ? -90.0f : faderToDb(f);
  db = constrain(roundf(db / step) * step + clicks * step, -90.0f, 10.0f);
  float nf = db <= -90.0f ? 0.0f : roundf(dbToFader(db) * 1023.0f) / 1023.0f;
  if (nf == f) nf = constrain(roundf(f * 1023.0f) + (clicks > 0 ? 1 : -1), 0.0f, 1023.0f) / 1023.0f;
  return nf;
}

void editParam(int clicks, bool fast) {
  uint8_t id = editId, idx = editIdx;
  if (id == P_FADER) {
    Strip &s = strips[selected];
    if (!s.haveFader) return;
    s.fader = stepFaderLaw(s.fader, clicks, fast);
    s.sendDue = s.needsFinal = true;
    s.lockedUntil = millis() + LOCK_MS;
    if (!touched) requestMove();  // the motor follows the knob
    dirty = true;
    return;
  }
  float v = paramValue(selected, id, idx);
  if (isnan(v)) return;
  const ParamInfo &p = PARAMS[id];
  switch (p.kind) {
    case 'F': v = stepFaderLaw(v, clicks, fast); break;
    case 'f': v = constrain(roundf(v * p.steps) + clicks * (fast ? 4 : 1), 0.0f, (float)p.steps) / p.steps; break;
    case 'i': v = constrain(v + clicks, 0.0f, (float)p.steps); break;
    case 'b': v = clicks > 0 ? 1.0f : 0.0f; break;
  }
  setParam(selected, id, idx, v);
}

// Encoder press: put the parameter being edited back to its default.
void resetParam() {
  uint8_t id = editId, idx = editIdx;
  if (id == P_NONE || !haveMixer) return;
  if (id == P_FADER) {
    Strip &s = strips[selected];
    if (!s.haveFader) return;
    s.fader = 0.75f;  // 0 dB
    s.sendDue = s.needsFinal = true;
    s.lockedUntil = millis() + LOCK_MS;
    if (!touched) requestMove();
    dirty = true;
    return;
  }
  static const float BAND_HZ[4] = {100.0f, 500.0f, 2000.0f, 8000.0f};
  float v;
  switch (id) {
    case P_SEND:
    case P_OTHER:      v = 0.75f; break;                                   // 0 dB
    case P_PAN:        v = 0.5f; break;                                    // centre
    case P_GAIN:       v = 12.0f / 72.0f; break;                           // 0 dB
    case P_HPF:        v = log10f(80.0f / 20.0f) / log10f(20.0f); break;  // 80 Hz
    case P_EQ_TYPE:    v = 2; break;                                       // PEQ
    case P_EQ_F:       v = log10f(BAND_HZ[idx & 3] / 20.0f) / 3.0f; break;
    case P_EQ_G:       v = 0.5f; break;                                    // 0 dB
    case P_EQ_Q:       v = logf(2.0f / 10.0f) / logf(0.03f); break;        // Q 2
    case P_GATE_THR:   v = 0.0f; break;                                    // -80 dB
    case P_GATE_RANGE: v = (40.0f - 3.0f) / 57.0f; break;                  // 40 dB
    case P_GATE_ATT:   v = 0.0f; break;                                    // 0 ms
    case P_GATE_HOLD:  v = logf(10.0f / 0.02f) / logf(100000.0f); break;   // 10 ms
    case P_GATE_REL:   v = logf(200.0f / 5.0f) / logf(800.0f); break;      // 200 ms
    case P_DYN_THR:    v = 1.0f; break;                                    // 0 dB
    case P_DYN_RATIO:  v = 5; break;                                       // 3:1
    case P_DYN_KNEE:   v = 0.0f; break;
    case P_DYN_ATT:    v = 10.0f / 120.0f; break;                          // 10 ms
    case P_DYN_REL:    v = logf(100.0f / 5.0f) / logf(800.0f); break;     // 100 ms
    case P_DYN_MGAIN:  v = 0.0f; break;                                    // 0 dB
    case P_DYN_MIX:    v = 1.0f; break;                                    // 100%
    default: return;  // on/off settings have no default to return to
  }
  const ParamInfo &p = PARAMS[id];
  if (p.kind == 'f') v = roundf(v * p.steps) / p.steps;
  setParam(selected, id, idx, v);
}

void toggleParam(uint8_t id, uint8_t idx) {
  if (!haveMixer || PARAMS[id].kind != 'b') return;
  float v = paramValue(selected, id, idx);
  if (isnan(v)) return;
  setParam(selected, id, idx, v > 0.5f ? 0.0f : 1.0f);
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
  if (a.type == UI_EDIT) {  // arg = param (P_NONE to stop), arg2 = band/send
    editId = a.arg;
    editIdx = a.arg2;
    return;
  }
  if (a.type == UI_BANK) {
    setBank(a.arg2);
    return;
  }
  if (a.type == UI_SET) {  // set a parameter of the selected strip to a value (arg2)
    if (haveMixer && a.arg == P_INS_SLOT && selected < 16) {
      setParam(selected, P_INS_SLOT, 0, a.arg2);
      if (a.arg2 && details[selected].insOn != 1) setParam(selected, P_INS_ON, 0, 1);  // inserting = on
    }
    return;
  }
  if (a.type == UI_TOGGLE) {
    toggleParam(a.arg, a.arg2);
    return;
  }
  if (!haveMixer) return;
  char addr[32];
  if (a.type == UI_MUTE_ANY) {  // arg2: 0-17 a channel strip, 32+ one of others[]
    int8_t *on;
    if (a.arg2 < NUM_STRIPS) {
      on = &strips[a.arg2].on;
      snprintf(addr, sizeof(addr), "%s/mix/on", strips[a.arg2].prefix);
    } else if (a.arg2 >= 32 && a.arg2 < 32 + NUM_OTHERS) {
      Other &o = others[a.arg2 - 32];
      on = &o.on;
      snprintf(addr, sizeof(addr), "%s%s", o.prefix, a.arg2 - 32 >= OTHER_DCA ? "/on" : "/mix/on");
    } else return;
    if (*on < 0) return;
    *on = *on ? 0 : 1;
    sendOsc(mixerIp, addr, 'i', *on);
    dirty = true;
    return;
  }
  if (a.type == UI_MUTEGRP) {
    int g = a.arg2 & 3;
    if (mixInfo.muteGrp[g] < 0) return;
    mixInfo.muteGrp[g] = mixInfo.muteGrp[g] ? 0 : 1;
    snprintf(addr, sizeof(addr), "/config/mute/%d", g + 1);
    sendOsc(mixerIp, addr, 'i', mixInfo.muteGrp[g]);
    return;
  }
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

// ---------- 8-fader board ----------

// A second ESP32 (xr_fader_board) runs 8 motorized faders. The home screen's
// bank tabs choose what they control; this loop sends each fader the level
// of what it's assigned to, and turns a hand moving one into a level change.

const int BANK_FADERS = 8;
enum BankId : uint8_t { BANK_1_8, BANK_9_16, BANK_RTN, BANK_BUS, BANK_DCA, NUM_BANKS };
const uint32_t FB_TIMEOUT_MS = 1000;      // no status lines for this long = board gone
const uint32_t FB_RESEND_MS = 2000;       // re-send every target this often (lost lines)
const float    FB_DEADBAND = 4.0f / 1000; // ignore position wobble smaller than this
const uint32_t FB_OTHER_GAP_MS = 25;      // max send rate for bus/FX/DCA levels

struct BankItem { int8_t kind, idx; };    // kind: 0 nothing, 1 strip, 2 others[]
BankItem bankItem(int b, int i);
float bankValue(const BankItem &it);
void applyBankFader(int i, const BankItem &it, float pos, uint32_t now);

volatile uint8_t bank = BANK_1_8;
// Up to two fader boards, each on its own link and driving its own run of
// bank faders: one board with 8 faders (MPR121 touch) covers all of them;
// two boards with 4 each (touch on their own pins) cover 1-4 and 5-8. Both
// run the same firmware; the main board maps their fader numbers.
struct FaderLink {
  HardwareSerial &ser;
  uint8_t rx, tx;
  int first;                     // bank fader that this board's fader 0 is
  bool connected = false, hasTouch = false;
  int faders = 0;                // from its hello line
  uint32_t lastLine = 0, rxBytes = 0;
  uint16_t touch = 0;
  char buf[96];
  int len = 0;
};
FaderLink links[] = {{Serial1, FB_RX, FB_TX, 0}, {Serial2, FB2_RX, FB2_TX, FB2_FIRST}};
const int NUM_LINKS = sizeof(links) / sizeof(links[0]);
void handleBoardLine(FaderLink &k, char *line, uint32_t now);  // own prototype: the IDE's would precede FaderLink

// All boards together, as the UI and bank logic see them.
bool fbConnected = false, fbHasTouch = false;
int fbFaders = 0;
uint32_t fbLastResend = 0;
float fbPos[BANK_FADERS];        // where each fader is, 0-1 (-1 unknown)
uint16_t fbTouch = 0, fbWasTouched = 0;
float fbSent[BANK_FADERS];       // last target sent (-2 = force a send)
float fbApplied[BANK_FADERS];    // last position turned into a level
uint32_t fbOtherSentAt[BANK_FADERS];
uint32_t fbInfoHash[BANK_FADERS];  // last label/name/mute sent to each fader's display (0 = resend)

BankItem bankItem(int b, int i) {
  switch (b) {
    case BANK_1_8:  return {1, (int8_t)i};
    case BANK_9_16: return {1, (int8_t)(8 + i)};
    case BANK_RTN: {  // AUX, FX returns 1-4, MAIN
      static const BankItem RTN[] = {{1, AUX_STRIP}, {2, OTHER_RTN}, {2, OTHER_RTN + 1}, {2, OTHER_RTN + 2},
                                     {2, OTHER_RTN + 3}, {1, MAIN_STRIP}, {0, 0}, {0, 0}};
      return RTN[i];
    }
    case BANK_BUS:  // buses 1-6, FX sends 1-2
      return i < 6 ? BankItem{2, (int8_t)(OTHER_BUS + i)} : BankItem{2, (int8_t)(OTHER_FXSEND + i - 6)};
    case BANK_DCA:  // DCAs 1-4, FX sends 1-4
      return i < 4 ? BankItem{2, (int8_t)(OTHER_DCA + i)} : BankItem{2, (int8_t)(OTHER_FXSEND + i - 4)};
  }
  return {0, 0};
}

float bankValue(const BankItem &it) {
  if (it.kind == 1) return strips[it.idx].haveFader ? strips[it.idx].fader : NAN;
  if (it.kind == 2) return others[it.idx].fader;
  return NAN;
}

void setBank(int b) {
  if (b < 0 || b >= NUM_BANKS || b == bank) return;
  bank = b;
  for (float &x : fbSent) x = -2;  // every fader flies to its new level
  Serial.printf("[%lu] fader bank %d\n", millis(), b);
}

// The board driving bank fader i (an earlier board wins an overlap), or -1.
int linkFor(int i) {
  for (int l = 0; l < NUM_LINKS; l++) {
    const FaderLink &k = links[l];
    if (k.connected && i >= k.first && i < k.first + max(k.faders, 1)) return l;
  }
  return -1;
}

void faderBoardBegin() {
  for (int i = 0; i < BANK_FADERS; i++) { fbPos[i] = -1; fbSent[i] = -2; fbApplied[i] = -1; }
  for (FaderLink &k : links) {
    k.ser.begin(FB_BAUD, SERIAL_8N1, k.rx, k.tx);
    k.ser.println("?");
  }
  gpio_pullup_en((gpio_num_t)FB2_RX);  // nothing plugged in: idle high, not noise
}

void handleBoardLine(FaderLink &k, char *line, uint32_t now) {
  if (line[0] == 'S') {
    int v[BANK_FADERS], mask;
    if (sscanf(line + 1, "%d %d %d %d %d %d %d %d %d", &mask, &v[0], &v[1], &v[2], &v[3], &v[4],
               &v[5], &v[6], &v[7]) == 9) {
      k.touch = mask;
      if (!k.connected) {
        k.connected = true;
        for (int i = k.first; i < BANK_FADERS; i++) fbSent[i] = -2, fbInfoHash[i] = 0;  // send it everything
        k.ser.println("?");
      }
      k.lastLine = now;
      for (int n = 0; n < max(k.faders, 1) && k.first + n < BANK_FADERS; n++)
        if (linkFor(k.first + n) == &k - links) fbPos[k.first + n] = v[n] < 0 ? -1.0f : v[n] / 1000.0f;
    }
  } else if (line[0] == 'H') {
    int n = 0, t = 0;
    sscanf(line + 1, "%d %d", &n, &t);
    k.faders = n;
    k.hasTouch = t;
    Serial.printf("Fader board %d: %d faders (bank faders %d-%d), touch sensor %s\n", (int)(&k - links) + 1, n,
                  k.first + 1, min(k.first + n, BANK_FADERS), t ? "yes" : "no");
  }
}

void readFaderBoard(uint32_t now) {
  static uint32_t lastDebug = 0;  // DEBUG: link status every 3 s
  if (now - lastDebug >= 3000) {
    lastDebug = now;
    Serial.printf("[%lu] fader links:", now);
    for (int l = 0; l < NUM_LINKS; l++)
      Serial.printf(" %d: rx %lu bytes, %s;", l + 1, links[l].rxBytes, links[l].connected ? "connected" : "no board");
    Serial.println();
  }
  for (FaderLink &k : links) {
    while (k.ser.available()) {
      char c = k.ser.read();
      k.rxBytes++;
      if (c != '\n' && c != '\r') {
        if (k.len < (int)sizeof(k.buf) - 1) k.buf[k.len++] = c;
        continue;
      }
      if (!k.len) continue;
      k.buf[k.len] = 0;
      k.len = 0;
      handleBoardLine(k, k.buf, now);
    }
    if (k.connected && now - k.lastLine > FB_TIMEOUT_MS) {
      k.connected = false;
      k.touch = 0;
      for (int i = k.first; i < k.first + max(k.faders, 1) && i < BANK_FADERS; i++) fbPos[i] = -1;
      Serial.printf("Fader board %d disconnected\n", (int)(&k - links) + 1);
    }
  }
  // Combine the boards.
  fbConnected = fbHasTouch = false;
  fbFaders = 0;
  fbTouch = 0;
  for (const FaderLink &k : links) {
    if (!k.connected) continue;
    fbConnected = true;
    fbHasTouch |= k.hasTouch;
    fbFaders = max(fbFaders, min(k.first + max(k.faders, 1), BANK_FADERS));
  }
  for (int i = 0; i < fbFaders; i++) {
    int l = linkFor(i);
    if (l >= 0 && (links[l].touch & (1 << (i - links[l].first)))) fbTouch |= 1 << i;
  }
}

// A hand moved bank fader i: set what it's assigned to.
void applyBankFader(int i, const BankItem &it, float pos, uint32_t now) {
  pos = roundf(pos * 1023.0f) / 1023.0f;
  if (it.kind == 1) {
    Strip &s = strips[it.idx];
    s.fader = pos;
    s.sendDue = s.needsFinal = true;
    s.lockedUntil = now + LOCK_MS;
    if (it.idx == selected && !touched) requestMove();  // the console's own fader follows
    // A linked channel (stereo pair): the mixer moves its partner too, but
    // its updates arrive late and in steps. Move the partner's level here
    // straight away, so its fader glides along, and ignore the echoes.
    if (it.idx < 16 && mixInfo.chLink[it.idx / 2] == 1) {
      Strip &p = strips[it.idx ^ 1];
      p.fader = pos;
      p.lockedUntil = now + LOCK_MS;
      if ((it.idx ^ 1) == selected && !touched) requestMove();
    }
    dirty = true;
  } else if (it.kind == 2 && now - fbOtherSentAt[i] >= FB_OTHER_GAP_MS) {
    fbOtherSentAt[i] = now;
    setParam(selected, P_OTHER, it.idx, pos);
  } else {
    return;  // rate-limited: try again next loop
  }
  fbApplied[i] = pos;
  fbSent[i] = pos;  // it's already there; no need to send it back
}

// Tell the fader board what each bank fader is assigned to (label, name,
// mute, solo), for the displays above the faders: whenever it changes, and with
// the periodic re-send in case a line was lost.

void sendBankInfo(bool resend) {
  for (int i = 0; i < BANK_FADERS && i < max(fbFaders, 1); i++) {
    int l = linkFor(i);
    if (l < 0) continue;
    BankItem it = bankItem(bank, i);
    const char *label = "", *name = "";
    int8_t on = 1, solo = 0;
    if (it.kind == 1) {
      const Strip &st = strips[it.idx];
      label = st.label; name = st.name; on = st.on;
      solo = it.idx != MAIN_STRIP && details[it.idx].solo == 1;  // only channels and AUX report solo
    } else if (it.kind == 2) {
      const Other &o = others[it.idx];
      label = o.label; name = o.name; on = o.on;
    }
    char line[48];
    snprintf(line, sizeof(line), "N %d %d %d %s|%s", i - links[l].first, on != 0, solo, label, name);
    uint32_t h = 2166136261u;  // FNV-1a
    for (const char *c = line; *c; c++) h = (h ^ (uint8_t)*c) * 16777619u;
    h |= 1;
    if (h != fbInfoHash[i] || resend) {
      links[l].ser.println(line);
      fbInfoHash[i] = h;
    }
  }
}

void bankService(uint32_t now) {
  if (!fbConnected) return;
  bool resend = now - fbLastResend > FB_RESEND_MS;
  if (resend) fbLastResend = now;
  sendBankInfo(resend);
  for (int i = 0; i < BANK_FADERS && i < max(fbFaders, 1); i++) {
    BankItem it = bankItem(bank, i);
    bool hand = fbTouch & (1 << i);
    bool wasHand = fbWasTouched & (1 << i);

    if (hand && it.kind) {
      if (!wasHand && it.kind == 1 && it.idx != selected) selectStrip(it.idx);  // show it on the TFT
      float pos = fbPos[i];
      if (pos >= 0 && (fabsf(pos - fbApplied[i]) > FB_DEADBAND || !wasHand)) applyBankFader(i, it, pos, now);
      continue;
    }
    // Not touched: keep the fader on its level (unassigned faders park at the bottom)
    float v = it.kind ? bankValue(it) : 0.0f;
    if (isnan(v)) continue;
    int l = linkFor(i);
    if (l < 0) continue;  // no board drives this fader
    if (wasHand || resend || fabsf(v - fbSent[i]) > 0.002f) {
      links[l].ser.printf("T %d %d\n", i - links[l].first, (int)lroundf(v * 1000.0f));
      fbSent[i] = v;
      fbApplied[i] = v;
    }
  }
  fbWasTouched = fbTouch;
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
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), onEncoder, CHANGE);
  analogReadResolution(12);
  calibrateTouch();

  uiBegin();  // TFT splash, touchscreen calibration, then the UI task
  faderBoardBegin();

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
  debugCommands(now);
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
      // Everything else (routing, buses, FX, ...) on connect and every minute
      if (extraPos < 0 && (lastExtra == 0 || now - lastExtra > EXTRA_EVERY_MS)) {
        extraPos = 0;
        lastExtra = now;
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

  uint32_t clickGap;
  int clicks = takeClicks(clickGap);
  if (clicks) {
    if (editId != P_NONE && haveMixer) editParam(clicks, clickGap < 40);
    else selectStrip(selected + clicks);
  }
  if (encButtonPressed()) resetParam();

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

  readFaderBoard(now);
  if (online && haveMixer) bankService(now);

  for (Strip &s : strips) {
    if (!online) break;
    if (s.sendDue && now - s.lastSendAt >= SEND_GAP_MS) {
      s.lastSendAt = now;
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
