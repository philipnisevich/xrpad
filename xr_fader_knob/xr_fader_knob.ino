// XR18 fader knob — LCKFB ESP32-S3 core board (N8R8) over Wi-Fi/OSC.
//
// Turn the encoder to move the selected channel's fader, press it to step
// through CH 1-16, AUX and MAIN. The OLED shows the selected channel's
// number, name and level, plus a bar for every channel's fader.
//
// Wiring (everything on 3V3, never 5V):
//   OLED (SSD1306 128x64 I2C)  VCC -> 3V3, GND -> GND, SDA -> G08, SCL -> G09
//   Encoder (KY-040 module)    +   -> 3V3, GND -> GND, CLK -> G06, DT -> G05, SW -> G04

#include <WiFi.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>
#include "secrets.h"

const uint8_t  ENC_CLK      = 6;
const uint8_t  ENC_DT       = 5;
const uint8_t  ENC_SW       = 4;
const uint8_t  OLED_SDA     = 8;
const uint8_t  OLED_SCL     = 9;
const bool     ENC_REVERSE  = false;  // flip if clockwise turns the fader down

const uint16_t XR_PORT      = 10024;
const uint32_t XREMOTE_MS   = 8000;   // mixer drops subscriptions after 10 s
const uint32_t TIMEOUT_MS   = 20000;  // no packets for this long = lost mixer
const uint32_t QUERY_GAP_MS = 15;     // spacing between state queries
const uint32_t SEND_GAP_MS  = 25;     // max fader update rate while turning
const uint32_t LOCK_MS      = 600;    // ignore mixer echoes this long after a turn
const uint32_t FAST_TURN_MS = 50;     // clicks closer than this = 0.5 dB steps
const float    STEP_DB      = 0.1f;
const float    FAST_STEP_DB = 0.5f;
const uint32_t DEBOUNCE_MS  = 30;
const uint32_t FRAME_MS     = 33;

// ---------- channel strips ----------

struct Strip {
  char prefix[10];   // OSC prefix, e.g. "/ch/01"
  char label[7];     // shown in the header, e.g. "CH 1"
  char name[16];     // channel name from the mixer
  float fader;       // 0.0 - 1.0
  bool haveFader;
  bool sendDue;      // local change not yet sent
  bool needsFinal;   // resend + confirm once turning stops
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

float dbToFader(float db) {
  float f;
  if (db >= -10.0f)      f = (db + 30.0f) / 40.0f;
  else if (db >= -30.0f) f = (db + 50.0f) / 80.0f;
  else if (db >= -60.0f) f = (db + 70.0f) / 160.0f;
  else                   f = (db + 90.0f) / 480.0f;
  f = constrain(f, 0.0f, 1.0f);
  return roundf(f * 1023.0f) / 1023.0f;  // mixer resolution
}

// ---------- network state ----------

WiFiUDP udp;
IPAddress mixerIp;
bool haveMixer = false;
uint32_t lastRx = 0, lastXremote = 0, lastProbe = 0, lastQuery = 0, lastSend = 0;
int sweepPos = -1;  // position in the name/fader query sweep, -1 = idle
bool dirty = true;  // display needs redrawing

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
      if (millis() < s.lockedUntil) return;  // we're turning it; ignore echoes
      uint32_t raw = readBE32(arg);
      float f;
      memcpy(&f, &raw, 4);
      if (!s.haveFader || f != s.fader) dirty = true;
      s.fader = f;
      s.haveFader = true;
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
// wiggle or bounce never counts, and the gap between clicks is measured
// precisely instead of at the (display-bound) loop rate.
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

bool buttonPressed() {
  static bool stable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  bool r = digitalRead(ENC_SW);
  if (r != lastRead) { lastRead = r; changedAt = millis(); }
  if (millis() - changedAt > DEBOUNCE_MS && r != stable) {
    stable = r;
    return stable == LOW;  // fire on press, not release
  }
  return false;
}

void turnSelected(int clicks, float step) {
  Strip &s = strips[selected];
  if (!s.haveFader) return;
  float old = s.fader;
  float db = old <= 0.0f ? -90.0f : faderToDb(old);
  db = roundf(db / STEP_DB) * STEP_DB + clicks * step;  // keep levels on a 0.1 dB grid
  db = constrain(db, -90.0f, 10.0f);
  s.fader = db <= -90.0f ? 0.0f : dbToFader(db);
  // Low on the fader one mixer step is bigger than 0.1 dB; always move at
  // least one step so a click never does nothing.
  if (s.fader == old) {
    s.fader = constrain(roundf(old * 1023.0f) + (clicks > 0 ? 1 : -1), 0.0f, 1023.0f) / 1023.0f;
  }
  s.sendDue = s.needsFinal = true;
  s.lockedUntil = millis() + LOCK_MS;
  dirty = true;
}

// ---------- display ----------

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

void formatDb(char *out, size_t size, float f) {
  if (f <= 0.0f) { snprintf(out, size, "-inf dB"); return; }
  float db = faderToDb(f);
  snprintf(out, size, "%+.1f dB", db);
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

  // Header: channel number and level
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(0, 10, s.label);
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
  initStrips();

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), onEncoder, CHANGE);

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
      if (!strlen(XR18_IP)) haveMixer = false;  // rediscover
      dirty = true;
    }
  }

  if (buttonPressed()) {
    selected = (selected + 1) % NUM_STRIPS;
    dirty = true;
    Serial.printf("[%lu] button -> %s \"%s\"\n", now, strips[selected].label, strips[selected].name);
  }

  uint32_t gap;
  int clicks = takeClicks(gap);
  if (clicks && haveMixer) {
    float step = gap < FAST_TURN_MS ? FAST_STEP_DB : STEP_DB;
    turnSelected(clicks, step);
    char text[16];
    formatDb(text, sizeof(text), strips[selected].fader);
    Serial.printf("[%lu] encoder %+d x %.1f dB -> %s %s\n", now, clicks, step,
                  strips[selected].label, text);
  }

  for (Strip &s : strips) {
    if (s.sendDue && now - lastSend >= SEND_GAP_MS) {
      lastSend = now;
      s.sendDue = false;
      sendStripOsc(s, "/mix/fader", true);
    }
    // Once turning stops, send the final value again (in case a packet was
    // lost) and ask the mixer to confirm it.
    if (s.needsFinal && !s.sendDue && now >= s.lockedUntil) {
      s.needsFinal = false;
      sendStripOsc(s, "/mix/fader", true);
      sendStripOsc(s, "/mix/fader");
    }
  }

  updateDisplay();
  delay(1);
}
