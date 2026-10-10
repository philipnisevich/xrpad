// XR18 motorized fader — LCKFB ESP32-S3 core board (N8R8) over Wi-Fi/OSC.
//
// Turn the encoder to pick a channel (CH 1-16, AUX, MAIN). The motorized
// fader drives itself to that channel's level, and follows changes made from
// the app; moving it by hand sets the level. The OLED shows the selected
// channel's number, name and level, plus a bar for every channel's fader.
//
// If you grab the fader while the motor is moving it, the motor lets go and
// the fader falls back to pickup: it takes control once moved past the
// channel's level, with an arrow showing which way.
//
// Wiring (everything on 3V3, never 5V):
//   OLED (SSD1306 128x64 I2C)  VCC -> 3V3, GND -> GND, SDA -> G08, SCL -> G09
//   Encoder (KY-040 module)    +   -> 3V3, GND -> GND, CLK -> G06, DT -> G05
//   Slide fader (10k linear)   one end -> 3V3, other end -> GND, wiper -> G07,
//                              touch pin -> G10 (keep this wire short)
//   Motor driver (TC1508A)     + -> 5V, - -> GND, IN1 -> G12, IN2 -> G11,
//                              motor outputs (MOTOR-A) -> fader motor pins
//
// The fader only sends changes while the knob is being touched, so electrical
// noise on a fader that's sitting still never moves the mixer.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>
#include "secrets.h"

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

const uint32_t POT_MIN_MV   = 40;     // reading at the bottom of travel
const uint32_t POT_MAX_MV   = 3100;   // reading at the top (ADC tops out around 3.1 V)
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
const float    MOTOR_MIN_DUTY = 0.55f; // least power that still moves the fader
const float    MOTOR_KP     = 5.0f;   // full power until within 20% of travel
const float    MOTOR_TOL    = 0.008f; // close enough to stop
const uint32_t MOTOR_BRAKE_MS = 30;   // short brake on arrival to stop coasting
const uint32_t MOTOR_TIMEOUT_MS = 1500;

const uint16_t XR_PORT      = 10024;
const uint32_t XREMOTE_MS   = 8000;   // mixer drops subscriptions after 10 s
const uint32_t TIMEOUT_MS   = 20000;  // no packets for this long = lost mixer
const uint32_t QUERY_GAP_MS = 15;     // spacing between state queries
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
  bool sendDue;      // local change not yet sent
  bool needsFinal;   // resend + confirm once the fader stops
  uint32_t lockedUntil;
};

const int NUM_STRIPS = 18;
Strip strips[NUM_STRIPS];
int selected = 0;

void initStrips() {
  for (int i = 0; i < 16; i++) {
    snprintf(strips[i].prefix, sizeof(strips[i].prefix), "/ch/%02d", i + 1);
    snprintf(strips[i].label, sizeof(strips[i].label), "CH %d", i + 1);
  }
  strcpy(strips[16].prefix, "/rtn/aux");
  strcpy(strips[16].label, "AUX");
  strcpy(strips[17].prefix, "/lr");
  strcpy(strips[17].label, "MAIN");
}

// X-Air fader law: 0.0-1.0 maps to -inf..+10 dB in four linear segments.
float faderToDb(float f) {
  if (f >= 0.5f)    return f * 40.0f - 30.0f;
  if (f >= 0.25f)   return f * 80.0f - 50.0f;
  if (f >= 0.0625f) return f * 160.0f - 70.0f;
  return f * 480.0f - 90.0f;
}

// ---------- network state ----------

WiFiUDP udp;
IPAddress mixerIp;
bool haveMixer = false;
uint32_t lastRx = 0, lastXremote = 0, lastProbe = 0, lastQuery = 0, lastSend = 0;
int sweepPos = -1;  // position in the name/fader query sweep, -1 = idle
bool dirty = true;  // display needs redrawing

// ---------- slide fader state ----------

float potPos = 0.0f;      // fader position, 0.0 - 1.0, on the mixer's 1023-step grid
bool pickedUp = false;    // fader is in control of the selected channel
volatile bool touched = !TOUCH_ENABLED;
float touchBaseline = 0.0f;

// ---------- motor ----------

// The motor runs in its own task so it's controlled every millisecond; the
// main loop stalls ~35 ms on each display redraw, long enough for the motor
// to overshoot badly. The task also owns the ADC and publishes the filtered
// fader position for the main loop.

volatile float potSmooth = -1.0f;   // fader position 0.0 - 1.0, filtered
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
  float smooth = -1.0f, startErr = 0.0f;
  bool moving = false;
  uint32_t seenSeq = 0, startAt = 0, brakeUntil = 0;

  for (;;) {
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(POT_PIN);
    mv /= 4;
    float raw = constrain(((float)mv - POT_MIN_MV) / (POT_MAX_MV - POT_MIN_MV), 0.0f, 1.0f);
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
    }

    if (moving) {
      float err = motorTarget - smooth;
      if (touched) {
        motorDrive(0.0f);
        moving = false;
        Serial.printf("[%lu] motor: hand on fader, letting go\n", now);
      } else if (fabsf(err) <= MOTOR_TOL) {
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
        float power = constrain(fabsf(err) * MOTOR_KP, MOTOR_MIN_DUTY, 1.0f);
        motorDrive(err > 0.0f ? power : -power);
      }
    }
    vTaskDelay(1);
  }
}

void motorBegin() {
  ledcAttach(MOTOR_IN1, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(MOTOR_IN2, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  motorDrive(0.0f);
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

// ---------- OSC encoding ----------

size_t padTo4(size_t n) { return (n + 4) & ~3u; }  // includes the null terminator

size_t oscString(uint8_t *buf, const char *s) {
  size_t len = strlen(s), padded = padTo4(len);
  memset(buf, 0, padded);
  memcpy(buf, s, len);
  return padded;
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
  udp.beginPacket(ip, XR_PORT);
  udp.write(buf, n);
  udp.endPacket();
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
    if (!haveMixer) {
      mixerIp = from;
      haveMixer = true;
      lastXremote = 0;  // subscribe and start a query sweep right away
      dirty = true;
      Serial.printf("Found mixer at %s\n", from.toString().c_str());
    }
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
    }
    return;
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
  dirty = true;
  Serial.printf("[%lu] select %s \"%s\"\n", millis(), s.label, s.name);
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

// ---------- display ----------

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
  int mode = WiFi.status() != WL_CONNECTED ? 0 : !haveMixer ? 1 : 2;
  if (mode != lastMode) dirty = true;
  if (!dirty || millis() - lastDraw < FRAME_MS) return;
  lastDraw = millis();
  lastMode = mode;
  dirty = false;
  if (mode == 0)      drawStatus("Connecting", WIFI_SSID);
  else if (mode == 1) drawStatus("Finding XR18", "on the network...");
  else                drawMixer();
}

// ---------- main ----------

void setup() {
  Serial.begin(115200);
  // With USB plugged in but no serial monitor open, each print would block
  // for up to 100 ms waiting for a reader and freeze the display. Drop
  // output instead.
  Serial.setTxTimeoutMs(0);
  initStrips();

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), onEncoder, CHANGE);
  analogReadResolution(12);
  calibrateTouch();
  motorBegin();

  oled.setBusClock(400000);
  oled.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // lower latency
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    updateDisplay();
    delay(50);
  }
  Serial.printf("\nConnected, my IP is %s\n", WiFi.localIP().toString().c_str());
  udp.begin(XR_PORT);

  if (strlen(XR18_IP) && mixerIp.fromString(XR18_IP)) {
    haveMixer = true;
    Serial.printf("Using mixer at %s\n", XR18_IP);
  }
}

void loop() {
  uint32_t now = millis();

  int size;
  while ((size = udp.parsePacket()) > 0) {
    uint8_t buf[256];
    int n = udp.read(buf, sizeof(buf));
    lastRx = now;
    handlePacket(buf, n, udp.remoteIP());
  }

  if (!haveMixer) {
    if (now - lastProbe > 1000) {  // broadcast discovery
      lastProbe = now;
      sendOsc(IPAddress(255, 255, 255, 255), "/xinfo");
    }
  } else {
    if (lastXremote == 0 || now - lastXremote > XREMOTE_MS) {
      lastXremote = now;
      sendOsc(mixerIp, "/xremote");  // subscribe to changes from apps/scenes
      if (sweepPos < 0) sweepPos = 0;  // re-read every name and fader
    }

    // Spread queries out so replies don't overflow the receive buffer
    if (sweepPos >= 0 && now - lastQuery >= QUERY_GAP_MS) {
      lastQuery = now;
      const Strip &s = strips[sweepPos / 2];
      sendStripOsc(s, sweepPos % 2 ? "/mix/fader" : "/config/name");
      if (++sweepPos >= NUM_STRIPS * 2) sweepPos = -1;
    }

    if (lastRx && now - lastRx > TIMEOUT_MS) {
      Serial.println("Lost contact with mixer");
      lastRx = 0;
      for (Strip &s : strips) s.haveFader = false;
      pickedUp = false;
      if (!strlen(XR18_IP)) haveMixer = false;  // rediscover
      dirty = true;
    }
  }

  int clicks = takeClicks();
  if (clicks) selectStrip(selected + clicks);

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

  updateDisplay();
  delay(1);
}
