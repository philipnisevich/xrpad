// 4.0" 480x320 ST7796 TFT with XPT2046 touch: the selected channel in detail
// plus an overview of every strip. Runs in its own task on core 0 so drawing
// never holds up the fader, motor or network. Each panel is drawn into an
// off-screen sprite and pushed only when what it shows has changed.
//
// Included from xr_console.ino after the state it reads (strips, details,
// meters, history), which it copies under stateLock once per frame.

#pragma once
#include <LovyanGFX.hpp>
#include <Preferences.h>

namespace ui {

const int PIN_SCK = 18, PIN_MOSI = 17, PIN_MISO = 16;
const int PIN_CS = 15, PIN_DC = 41, PIN_RST = 42, PIN_BL = 40;
const int PIN_TOUCH_CS = 39, PIN_TOUCH_IRQ = 38;
const uint8_t ROTATION = 3;          // landscape, pins on the right
const uint32_t FRAME_MS = 30;
const uint32_t PRESS_FLASH_MS = 150; // how long a tapped button lights up

class Tft : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 panel;
  lgfx::Bus_SPI bus;
  lgfx::Touch_XPT2046 touch;

 public:
  Tft() {
    auto b = bus.config();
    b.spi_host = SPI2_HOST;
    b.spi_mode = 0;
    b.freq_write = 40000000;
    b.freq_read = 16000000;
    b.pin_sclk = PIN_SCK;
    b.pin_mosi = PIN_MOSI;
    b.pin_miso = PIN_MISO;
    b.pin_dc = PIN_DC;
    bus.config(b);
    panel.setBus(&bus);

    auto p = panel.config();
    p.pin_cs = PIN_CS;
    p.pin_rst = PIN_RST;
    p.panel_width = 320;
    p.panel_height = 480;
    p.readable = false;
    p.bus_shared = true;  // touch is on the same SPI bus
    panel.config(p);

    auto t = touch.config();
    t.spi_host = SPI2_HOST;
    t.freq = 1000000;
    t.pin_sclk = PIN_SCK;
    t.pin_mosi = PIN_MOSI;
    t.pin_miso = PIN_MISO;
    t.pin_cs = PIN_TOUCH_CS;
    t.pin_int = PIN_TOUCH_IRQ;
    t.bus_shared = true;
    touch.config(t);
    panel.setTouch(&touch);

    setPanel(&panel);
  }
};

Tft tft;

// ---------- colours ----------

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

const uint16_t C_BG     = rgb(10, 13, 18);
const uint16_t C_PANEL  = rgb(21, 27, 36);
const uint16_t C_GRAPH  = rgb(14, 18, 25);
const uint16_t C_BORDER = rgb(44, 54, 68);
const uint16_t C_GRID   = rgb(36, 45, 58);
const uint16_t C_GRID0  = rgb(72, 85, 102);
const uint16_t C_TEXT   = rgb(232, 238, 244);
const uint16_t C_DIM    = rgb(122, 136, 152);
const uint16_t C_FAINT  = rgb(70, 82, 98);
const uint16_t C_GREEN  = rgb(34, 197, 94);
const uint16_t C_AMBER  = rgb(245, 158, 11);
const uint16_t C_RED    = rgb(239, 68, 68);
const uint16_t C_TEAL   = rgb(45, 212, 191);
const uint16_t C_ORANGE = rgb(251, 146, 60);
const uint16_t C_PURPLE = rgb(167, 139, 250);
const uint16_t C_MUTED  = rgb(72, 14, 16);

const uint16_t BAND_COLORS[4] = {rgb(248, 113, 113), rgb(251, 191, 36), rgb(74, 222, 128),
                                 rgb(96, 165, 250)};

uint16_t mix565(uint16_t a, uint16_t b, float t) {  // t = 0 -> a, 1 -> b
  int ar = a >> 11, ag = (a >> 5) & 63, ab = a & 31;
  int br = b >> 11, bg = (b >> 5) & 63, bb = b & 31;
  return (uint16_t)((int)(ar + (br - ar) * t) << 11 | (int)(ag + (bg - ag) * t) << 5 |
                    (int)(ab + (bb - ab) * t));
}

// X-Air scribble-strip colours (8-15 are the inverted versions of 0-7).
uint16_t chanColor(int8_t c) {
  static const uint16_t COLORS[8] = {
    rgb(120, 134, 150), rgb(239, 68, 68), rgb(34, 197, 94), rgb(234, 179, 8),
    rgb(59, 130, 246), rgb(217, 70, 239), rgb(6, 182, 212), rgb(226, 232, 240),
  };
  return c < 0 ? rgb(96, 165, 250) : COLORS[c & 7];
}

uint16_t meterColor(float db) {
  return db > -6.0f ? C_RED : db > -18.0f ? C_AMBER : C_GREEN;
}

// ---------- value conversions (X-Air 0.0-1.0 parameters) ----------

float eqFreq(float x)  { return 20.0f * powf(1000.0f, x); }   // 20 Hz - 20 kHz
float eqGain(float x)  { return x * 30.0f - 15.0f; }          // -15..+15 dB
float eqQ(float x)     { return 10.0f * powf(0.03f, x); }     // 10..0.3
float hpfFreq(float x) { return 20.0f * powf(20.0f, x); }     // 20..400 Hz
float preampDb(float x){ return x * 72.0f - 12.0f; }          // -12..+60 dB
float gateDb(float x)  { return x * 80.0f - 80.0f; }          // -80..0 dB
float dynDb(float x)   { return x * 60.0f - 60.0f; }          // -60..0 dB
const float RATIOS[12] = {1.1f, 1.3f, 1.5f, 2, 2.5f, 3, 4, 5, 7, 10, 20, 100};

void faderText(char *out, size_t size, float f) {
  if (f <= 0.0f) snprintf(out, size, "-inf");
  else           snprintf(out, size, "%+.1f", faderToDb(f));
}

// Magnitude (dB) at `freq` of one RBJ-cookbook biquad: how the EQ curve is drawn.
float biquadDb(int type, float f0, float gain, float q, float freq) {
  const float fs = 48000.0f;
  float w0 = 2.0f * PI * f0 / fs, cw = cosf(w0), sw = sinf(w0);
  float alpha = sw / (2.0f * q), A = powf(10.0f, gain / 40.0f), sA = 2.0f * sqrtf(A) * alpha;
  float b0, b1, b2, a0, a1, a2;
  switch (type) {
    case 0:  // low cut
      b0 = (1 + cw) / 2; b1 = -(1 + cw); b2 = b0; a0 = 1 + alpha; a1 = -2 * cw; a2 = 1 - alpha;
      break;
    case 1:  // low shelf
      b0 = A * ((A + 1) - (A - 1) * cw + sA); b1 = 2 * A * ((A - 1) - (A + 1) * cw);
      b2 = A * ((A + 1) - (A - 1) * cw - sA); a0 = (A + 1) + (A - 1) * cw + sA;
      a1 = -2 * ((A - 1) + (A + 1) * cw); a2 = (A + 1) + (A - 1) * cw - sA;
      break;
    case 4:  // high shelf
      b0 = A * ((A + 1) + (A - 1) * cw + sA); b1 = -2 * A * ((A - 1) + (A + 1) * cw);
      b2 = A * ((A + 1) + (A - 1) * cw - sA); a0 = (A + 1) - (A - 1) * cw + sA;
      a1 = 2 * ((A - 1) - (A + 1) * cw); a2 = (A + 1) - (A - 1) * cw - sA;
      break;
    case 5:  // high cut
      b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = b0; a0 = 1 + alpha; a1 = -2 * cw; a2 = 1 - alpha;
      break;
    default:  // 2 PEQ, 3 VEQ: peaking
      b0 = 1 + alpha * A; b1 = -2 * cw; b2 = 1 - alpha * A;
      a0 = 1 + alpha / A; a1 = -2 * cw; a2 = 1 - alpha / A;
      break;
  }
  float w = 2.0f * PI * freq / fs, c1 = cosf(w), s1 = sinf(w), c2 = cosf(2 * w), s2 = sinf(2 * w);
  float nr = b0 + b1 * c1 + b2 * c2, ni = -(b1 * s1 + b2 * s2);
  float dr = a0 + a1 * c1 + a2 * c2, di = -(a1 * s1 + a2 * s2);
  return 10.0f * log10f((nr * nr + ni * ni) / (dr * dr + di * di));
}

// ---------- snapshot of the shared state ----------

struct StripView {
  char label[7], name[16];
  float fader, meter;
  bool haveFader;
  int8_t on, color;
};

struct Snap {
  bool calibrating;  // fader calibration running at boot
  int sel;
  StripView s[NUM_STRIPS];
  Detail d;
  int8_t hist[HIST_LEN];
  uint32_t meterSeq;
  bool pickedUp, faderTouched;
  float faderPos;  // where the physical fader is

  NetState net;
  bool online, scanning, haveMixer;
  char netSsid[33];
  ScanEntry scan[MAX_SCAN];
  int scanCount;
  uint32_t scanSeq;
  char saved[MAX_SAVED][33];
  int savedCount;
  int8_t rssi;
  char ip[16], mixer[16];

  // Loading progress after the mixer is found
  uint32_t syncStart;
  int namesKnown, fadersKnown, mutesKnown;
  float detailFrac;
  bool metersLive;
  char mixerName[24], mixerModel[16];
};

Snap snap;

void takeSnapshot() {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  snap.calibrating = bootStage == 0;
  snap.sel = selected;
  for (int i = 0; i < NUM_STRIPS; i++) {
    StripView &v = snap.s[i];
    memcpy(v.label, strips[i].label, sizeof(v.label));
    memcpy(v.name, strips[i].name, sizeof(v.name));
    v.fader = strips[i].fader;
    v.haveFader = strips[i].haveFader;
    v.on = strips[i].on;
    v.color = details[i].color;
    v.meter = meterDb[i];
  }
  snap.d = details[selected];
  for (int x = 0; x < HIST_LEN; x++) snap.hist[x] = hist[selected][(histHead + x) % HIST_LEN];
  snap.meterSeq = meterSeq;
  snap.pickedUp = pickedUp;
  snap.faderTouched = touched && TOUCH_ENABLED;
  snap.faderPos = potSmooth;

  snap.net = netState;
  snap.online = netState == NET_CONNECTED && WiFi.status() == WL_CONNECTED;
  snap.scanning = scanning;
  snap.haveMixer = haveMixer;
  memcpy(snap.netSsid, netSsid, sizeof(snap.netSsid));
  snap.scanCount = scanCount;
  memcpy(snap.scan, scanList, sizeof(ScanEntry) * scanCount);
  snap.scanSeq = scanSeq;
  snap.savedCount = savedCount;
  for (int i = 0; i < savedCount; i++) memcpy(snap.saved[i], savedNets[i].ssid, 33);
  snap.rssi = snap.online ? WiFi.RSSI() : 0;
  strlcpy(snap.ip, snap.online ? WiFi.localIP().toString().c_str() : "", sizeof(snap.ip));
  strlcpy(snap.mixer, haveMixer ? mixerIp.toString().c_str() : "", sizeof(snap.mixer));

  snap.syncStart = syncStart;
  snap.namesKnown = snap.fadersKnown = snap.mutesKnown = 0;
  for (const Strip &st : strips) {
    snap.namesKnown += st.nameKnown;
    snap.fadersKnown += st.haveFader;
    snap.mutesKnown += st.on >= 0;
  }
  static int detailTotal = 0, detailFor = -1;
  if (detailFor != detailStrip) { detailFor = detailStrip; detailTotal = detailQueryCount(detailStrip); }
  snap.detailFrac = detailDone ? 1.0f
                  : detailPos < 0 || detailStrip != selected ? 0.0f
                  : constrain((float)detailPos / max(detailTotal, 1), 0.0f, 1.0f);
  snap.metersLive = !isnan(meterDb[0]);
  strlcpy(snap.mixerName, mixerName, sizeof(snap.mixerName));
  strlcpy(snap.mixerModel, mixerModel, sizeof(snap.mixerModel));
  xSemaphoreGive(stateLock);
}

bool isSaved(const char *ssid) {
  for (int i = 0; i < snap.savedCount; i++)
    if (!strcmp(snap.saved[i], ssid)) return true;
  return false;
}

// ---------- change detection ----------

struct Hash {
  uint32_t h = 2166136261u;
  template <class T> Hash &add(const T &v) {
    const uint8_t *p = (const uint8_t *)&v;
    for (size_t i = 0; i < sizeof(T); i++) { h ^= p[i]; h *= 16777619u; }
    return *this;
  }
  Hash &q(float v, float scale) { return add(isnan(v) ? INT32_MIN : (int32_t)lroundf(v * scale)); }
  Hash &str(const char *s) { while (*s) add(*s++); return *this; }
};

// ---------- layout ----------

struct Panel {
  int16_t x, y, w, h;
  LGFX_Sprite spr;
  uint32_t hash;
};

Panel pHeader  = {0, 0, 480, 52};
Panel pLevel   = {8, 58, 104, 204};
Panel pEq      = {118, 58, 236, 124};
Panel pHistory = {118, 188, 236, 74};
Panel pPreamp  = {360, 58, 112, 124};
Panel pSends   = {360, 188, 112, 74};
Panel pStrips  = {0, 268, 480, 52};
Panel *const PANELS[] = {&pHeader, &pLevel, &pEq, &pHistory, &pPreamp, &pSends, &pStrips};

const int16_t MUTE_X = 300, SOLO_X = 390, BTN_Y = 8, BTN_W = 82, BTN_H = 36;
const int16_t GEAR_X = 272, GEAR_Y = 26;  // settings gear in the header
const int16_t STRIP_X0 = 6, STRIP_PITCH = 26;

int pressedButton = -1;      // 0 mute, 1 solo, for the tap flash
uint32_t pressedAt = 0;

// Rounded panel with a small caption in the top-left corner.
void panelBase(LGFX_Sprite &s, const char *title) {
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString(title, 10, 7);
}

// Small rounded label; filled when on, outlined when off.
int pill(LGFX_Sprite &s, int xRight, int y, const char *text, uint16_t color, bool on) {
  s.setFont(&fonts::DejaVu9);
  int w = s.textWidth(text) + 10, x = xRight - w;
  if (on) {
    s.fillSmoothRoundRect(x, y, w, 13, 6, color);
    s.setTextColor(C_BG);
  } else {
    s.drawRoundRect(x, y, w, 13, 6, C_BORDER);
    s.setTextColor(C_FAINT);
  }
  s.setTextDatum(middle_center);
  s.drawString(text, x + w / 2, y + 7);
  return x - 4;  // where the next pill to the left should end
}

void dashedHLine(LGFX_Sprite &s, int x0, int x1, int y, uint16_t color) {
  for (int x = x0; x < x1; x += 6) s.drawFastHLine(x, y, min(3, x1 - x), color);
}

// Arc helper: angles in degrees clockwise from 3 o'clock, a0 < 360 <= a1 allowed.
void arc(LGFX_Sprite &s, int cx, int cy, int r0, int r1, float a0, float a1, uint16_t color) {
  if (a1 <= a0) return;
  if (a1 <= 360.0f) {
    s.fillArc(cx, cy, r0, r1, a0, a1, color);
  } else {
    s.fillArc(cx, cy, r0, r1, a0, 360.0f, color);
    s.fillArc(cx, cy, r0, r1, 0.0f, a1 - 360.0f, color);
  }
}

// Settings gear icon; works on the screen or a sprite.
template <class G>
void drawGear(G &g, int cx, int cy, uint16_t color, uint16_t bg) {
  for (int k = 0; k < 8; k++) {
    float a = k * PI / 4.0f;
    g.drawWideLine(cx + cosf(a) * 7, cy + sinf(a) * 7, cx + cosf(a) * 11, cy + sinf(a) * 11, 2.2f, color);
  }
  g.fillSmoothCircle(cx, cy, 8, color);
  g.fillSmoothCircle(cx, cy, 3, bg);
}

// ---------- panels ----------

void drawHeader(LGFX_Sprite &s) {
  const StripView &v = snap.s[snap.sel];
  const Detail &d = snap.d;
  uint16_t cc = chanColor(v.color);
  bool flash = millis() - pressedAt < PRESS_FLASH_MS;
  s.fillScreen(C_BG);

  s.fillSmoothRoundRect(10, 8, 6, 36, 3, cc);
  s.setTextDatum(top_left);
  s.setFont(&fonts::DejaVu12);
  s.setTextColor(C_DIM);
  s.drawString(v.label, 26, 6);
  int x = 26 + s.textWidth(v.label) + 10;
  if (d.lr >= 0) {
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(d.lr ? C_DIM : C_AMBER);
    s.drawString(d.lr ? "LR" : "NOT ON LR", x, 8);
  }

  const char *name = v.name[0] ? v.name : v.label;
  s.setFont(&fonts::FreeSansBold18pt7b);
  if (s.textWidth(name) > GEAR_X - 50) s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextColor(C_TEXT);
  s.setTextDatum(bottom_left);
  s.drawString(name, 24, 48);

  drawGear(s, GEAR_X, GEAR_Y, C_DIM, C_BG);

  // Mute
  bool muted = v.on == 0;
  uint16_t mFill = muted ? C_RED : C_PANEL;
  if (flash && pressedButton == 0) mFill = mix565(mFill, C_TEXT, 0.3f);
  s.fillSmoothRoundRect(MUTE_X, BTN_Y, BTN_W, BTN_H, 8, mFill);
  if (!muted) s.drawRoundRect(MUTE_X, BTN_Y, BTN_W, BTN_H, 8, C_BORDER);
  s.setFont(&fonts::FreeSansBold9pt7b);
  s.setTextDatum(middle_center);
  s.setTextColor(muted ? C_TEXT : v.on < 0 ? C_FAINT : C_DIM);
  s.drawString(muted ? "MUTED" : "MUTE", MUTE_X + BTN_W / 2, BTN_Y + BTN_H / 2 + 1);

  // Solo (MAIN has no solo switch)
  bool soloOk = snap.sel != MAIN_STRIP;
  bool solo = d.solo == 1;
  uint16_t sFill = solo ? C_AMBER : C_PANEL;
  if (flash && pressedButton == 1 && soloOk) sFill = mix565(sFill, C_TEXT, 0.3f);
  s.fillSmoothRoundRect(SOLO_X, BTN_Y, BTN_W, BTN_H, 8, sFill);
  if (!solo) s.drawRoundRect(SOLO_X, BTN_Y, BTN_W, BTN_H, 8, C_BORDER);
  s.setTextColor(solo ? C_BG : soloOk ? C_DIM : C_GRID);
  s.drawString("SOLO", SOLO_X + BTN_W / 2, BTN_Y + BTN_H / 2 + 1);
}

void drawLevel(LGFX_Sprite &s) {
  const StripView &v = snap.s[snap.sel];
  uint16_t cc = chanColor(v.color);
  panelBase(s, "LEVEL");
  const int top = 24, bottom = 168, h = bottom - top;

  // Segmented input meter, -60..0 dBFS
  const int segs = 36, segH = h / segs;
  for (int k = 0; k < segs; k++) {
    float segDb = -60.0f + 60.0f * (k + 1) / segs;
    uint16_t c = meterColor(segDb);
    bool lit = !isnan(v.meter) && v.meter >= segDb - 60.0f / segs;
    int y = bottom - (k + 1) * segH;
    s.fillRect(10, y + 1, 12, segH - 1, lit ? c : mix565(C_PANEL, c, 0.15f));
  }
  s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_FAINT);
  s.setTextDatum(top_center);
  s.drawString("IN", 16, bottom + 3);

  // Fader scale, track and cap
  static const int MARKS[] = {10, 5, 0, -5, -10, -20, -30, -50};
  s.setTextDatum(middle_right);
  for (int db : MARKS) {
    int y = bottom - (int)lroundf(dbToFader(db) * h);
    s.drawFastHLine(56, y, 6, db == 0 ? C_GRID0 : C_GRID);
    s.drawFastHLine(72, y, 6, db == 0 ? C_GRID0 : C_GRID);
    char t[5];
    snprintf(t, sizeof(t), db > 0 ? "+%d" : "%d", db);
    s.setTextColor(db == 0 ? C_DIM : C_FAINT);
    s.drawString(t, 52, y);
  }
  s.drawString("inf", 52, bottom);
  s.fillSmoothRoundRect(65, top - 2, 4, h + 4, 2, C_GRAPH);

  if (!snap.pickedUp && snap.faderPos >= 0.0f && v.haveFader) {  // where the hand-fader is
    int gy = bottom - (int)lroundf(snap.faderPos * h);
    s.drawRoundRect(52, gy - 7, 30, 14, 3, C_FAINT);
  }
  if (v.haveFader) {
    int y = bottom - (int)lroundf(v.fader * h);
    s.fillSmoothRoundRect(52, y - 7, 30, 14, 3, snap.faderTouched ? C_TEXT : rgb(176, 188, 202));
    s.fillRect(55, y - 1, 24, 2, cc);
    if (snap.faderTouched) s.drawRoundRect(51, y - 8, 32, 16, 4, cc);
  }

  // Readout
  char t[12];
  if (v.haveFader) faderText(t, sizeof(t), v.fader);
  else strcpy(t, "--");
  s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextDatum(middle_center);
  s.setTextColor(C_TEXT);
  s.drawString(t, 52, 184);
  s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_FAINT);
  s.drawString("dB", 52, 198);
}

void drawEq(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  panelBase(s, "EQ");
  int right = s.width() - 8;
  bool eqKnown = d.eqOn >= 0;
  bool hpKnown = d.hpOn >= 0 && !isnan(d.hpf);
  if (eqKnown) right = pill(s, right, 5, d.eqOn ? "ON" : "OFF", C_GREEN, d.eqOn);
  if (hpKnown) {
    char t[16];
    snprintf(t, sizeof(t), "LOW CUT %.0f Hz", hpfFreq(d.hpf));
    pill(s, right, 5, t, C_TEAL, d.hpOn);
  }

  const int gx = 8, gy = 22, gw = 220, gh = 94;
  s.fillSmoothRoundRect(gx, gy, gw, gh, 4, C_GRAPH);
  auto xFor = [&](float hz) { return gx + (int)lroundf(log10f(hz / 20.0f) / 3.0f * (gw - 1)); };
  auto yFor = [&](float db) { return gy + gh / 2 - (int)lroundf(constrain(db, -15.0f, 15.0f) * (gh / 2 - 3) / 15.0f); };

  static const int MINOR[] = {50, 200, 500, 2000, 5000};
  for (int hz : MINOR) s.drawFastVLine(xFor(hz), gy + 2, gh - 4, C_GRID);
  static const int MAJOR[] = {100, 1000, 10000};
  static const char *const MAJOR_T[] = {"100", "1k", "10k"};
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(bottom_center);
  for (int i = 0; i < 3; i++) {
    s.drawFastVLine(xFor(MAJOR[i]), gy + 2, gh - 4, C_BORDER);
    s.setTextColor(C_FAINT);
    s.drawString(MAJOR_T[i], xFor(MAJOR[i]), gy + gh - 2);
  }
  for (int db : {-12, -6, 6, 12}) s.drawFastHLine(gx + 2, yFor(db), gw - 4, C_GRID);
  int zeroY = yFor(0);
  s.drawFastHLine(gx + 2, zeroY, gw - 4, C_GRID0);
  s.setTextDatum(middle_left);
  s.drawString("+12", gx + 4, yFor(12));
  s.drawString("-12", gx + 4, yFor(-12));

  bool bandsKnown = eqKnown;
  for (const EqBand &b : d.eq)
    if (b.type < 0 || isnan(b.f) || isnan(b.g) || isnan(b.q)) bandsKnown = false;
  if (!bandsKnown && !hpKnown) {
    s.setTextDatum(middle_center);
    s.setTextColor(C_FAINT);
    s.drawString(eqKnown || snap.sel == MAIN_STRIP ? "" : "reading EQ...", gx + gw / 2, gy + gh / 2 - 12);
    return;
  }

  // Response curve: the four bands (when the EQ is on) plus the low cut.
  bool eqActive = bandsKnown && d.eqOn == 1;
  bool hpActive = hpKnown && d.hpOn == 1;
  uint16_t lineColor = eqActive || hpActive ? cc : C_FAINT;
  uint16_t fillColor = mix565(C_GRAPH, cc, 0.28f);
  int prevY = zeroY;
  for (int x = 0; x < gw; x++) {
    float hz = 20.0f * powf(1000.0f, (float)x / (gw - 1));
    float db = 0.0f;
    if (eqActive)
      for (const EqBand &b : d.eq) db += biquadDb(b.type, eqFreq(b.f), eqGain(b.g), eqQ(b.q), hz);
    if (hpActive) db += biquadDb(0, hpfFreq(d.hpf), 0, 0.707f, hz);
    int y = yFor(db);
    if (eqActive || hpActive) {
      if (y < zeroY) s.drawFastVLine(gx + x, y, zeroY - y, fillColor);
      else if (y > zeroY) s.drawFastVLine(gx + x, zeroY + 1, y - zeroY, fillColor);
    }
    if (x > 0) s.drawWideLine(gx + x - 1, prevY, gx + x, y, 1.0f, lineColor);
    prevY = y;
  }

  // Band handles
  if (bandsKnown) {
    for (int i = 0; i < 4; i++) {
      const EqBand &b = d.eq[i];
      bool cut = b.type == 0 || b.type == 5;
      int x = constrain(xFor(eqFreq(b.f)), gx + 7, gx + gw - 8);
      int y = cut ? zeroY : yFor(eqGain(b.g));
      uint16_t c = d.eqOn == 1 ? BAND_COLORS[i] : C_FAINT;
      s.fillSmoothCircle(x, y, 7, c);
      s.setTextColor(C_BG);
      s.setTextDatum(middle_center);
      char t[2] = {(char)('1' + i), 0};
      s.drawString(t, x, y + 1);
    }
  }
}

void drawHistory(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  panelBase(s, "INPUT");
  int right = s.width() - 8;
  char t[20];
  if (d.dynOn >= 0 && !isnan(d.dynThr)) {
    if (d.dynRatio >= 0 && d.dynRatio < 12)
      snprintf(t, sizeof(t), "COMP %.0f  %g:1", dynDb(d.dynThr), RATIOS[d.dynRatio]);
    else
      snprintf(t, sizeof(t), "COMP %.0f", dynDb(d.dynThr));
    right = pill(s, right, 5, t, C_ORANGE, d.dynOn == 1);
  }
  if (d.gateOn >= 0 && !isnan(d.gateThr)) {
    snprintf(t, sizeof(t), "GATE %.0f", gateDb(d.gateThr));
    pill(s, right, 5, t, C_TEAL, d.gateOn == 1);
  }

  const int gx = 8, gy = 22, gw = 220, gh = 46;
  s.fillSmoothRoundRect(gx, gy, gw, gh, 4, C_GRAPH);
  auto yFor = [&](float db) { return gy + gh - 1 - (int)lroundf((constrain(db, -60.0f, 0.0f) + 60.0f) / 60.0f * (gh - 2)); };
  for (int db : {-12, -24, -36, -48}) s.drawFastHLine(gx + 2, yFor(db), gw - 4, C_GRID);

  for (int x = 0; x < gw && x < HIST_LEN; x++) {
    int8_t v = snap.hist[x];
    if (v <= -60) continue;
    int y = yFor(v), bottom = gy + gh - 1;
    uint16_t c = meterColor(v);
    s.drawFastVLine(gx + x, y, bottom - y, mix565(C_GRAPH, c, 0.35f));
    s.drawFastVLine(gx + x, y, 2, c);
  }

  // Thresholds: where the gate opens and the compressor starts working.
  if (!isnan(d.gateThr) && gateDb(d.gateThr) > -60.0f)
    dashedHLine(s, gx + 2, gx + gw - 2, yFor(gateDb(d.gateThr)), d.gateOn == 1 ? C_TEAL : C_FAINT);
  if (!isnan(d.dynThr))
    dashedHLine(s, gx + 2, gx + gw - 2, yFor(dynDb(d.dynThr)), d.dynOn == 1 ? C_ORANGE : C_FAINT);
  (void)cc;
}

void drawPreamp(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  panelBase(s, "PREAMP");

  // Gain dial, -12..+60 dB over 270 degrees
  const int cx = 36, cy = 58;
  arc(s, cx, cy, 21, 27, 135.0f, 405.0f, C_GRAPH);
  s.setTextDatum(middle_center);
  if (!isnan(d.gain)) {
    arc(s, cx, cy, 21, 27, 135.0f, 135.0f + 270.0f * constrain(d.gain, 0.0f, 1.0f), cc);
    char t[8];
    snprintf(t, sizeof(t), "%+.0f", preampDb(d.gain));
    s.setFont(&fonts::FreeSansBold9pt7b);
    s.setTextColor(C_TEXT);
    s.drawString(t, cx, cy - 1);
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.drawString("dB", cx, cy + 13);
  } else {
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.drawString(snap.sel == MAIN_STRIP ? "n/a" : "--", cx, cy);
  }

  // Badges
  auto badge = [&](int y, const char *text, int8_t state, uint16_t color) {
    if (state < 0) return;
    if (state) s.fillSmoothRoundRect(70, y, 34, 16, 4, color);
    else s.drawRoundRect(70, y, 34, 16, 4, C_BORDER);
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(state ? C_BG : C_FAINT);
    s.setTextDatum(middle_center);
    s.drawString(text, 87, y + 8);
  };
  badge(26, "48V", d.phantom, C_RED);
  badge(46, "INV", d.invert, C_AMBER);
  badge(66, "LR", d.lr, rgb(96, 165, 250));

  // Pan
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(middle_left);
  s.setTextColor(C_DIM);
  s.drawString("PAN", 10, 98);
  if (!isnan(d.pan)) {
    int p = (int)lroundf(d.pan * 200.0f - 100.0f);
    char t[8];
    if (p == 0) strcpy(t, "C");
    else snprintf(t, sizeof(t), "%c%d", p < 0 ? 'L' : 'R', abs(p));
    s.setTextDatum(middle_right);
    s.setTextColor(C_TEXT);
    s.drawString(t, 102, 98);
  }
  const int bx = 10, bw = 92, by = 110;
  s.fillSmoothRoundRect(bx, by, bw, 5, 2, C_GRAPH);
  s.drawFastVLine(bx + bw / 2, by - 3, 11, C_GRID0);
  if (!isnan(d.pan)) {
    int px = bx + (int)lroundf(d.pan * bw);
    int mid = bx + bw / 2;
    s.fillRect(min(px, mid), by + 1, abs(px - mid), 3, cc);
    s.fillSmoothCircle(px, by + 2, 5, C_TEXT);
  }
}

void drawSends(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  panelBase(s, "SENDS");
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_right);
  s.setTextColor(C_FAINT);
  s.drawString("BUS / FX", s.width() - 10, 7);

  const int top = 22, h = 34, bw = 6;
  for (int k = 0; k < 10; k++) {
    bool fx = k >= 6;
    int x = 10 + k * 9 + (fx ? 6 : 0);
    s.fillSmoothRoundRect(x, top, bw, h, 2, C_GRAPH);
    s.drawFastHLine(x - 1, top + h - (int)lroundf(0.75f * h), bw + 2, C_GRID0);  // 0 dB
    if (!isnan(d.sends[k])) {
      int fh = (int)lroundf(constrain(d.sends[k], 0.0f, 1.0f) * h);
      if (fh > 0) s.fillSmoothRoundRect(x, top + h - fh, bw, fh, 2, fx ? C_PURPLE : C_TEAL);
    }
    char t[3];
    snprintf(t, sizeof(t), "%d", fx ? k - 5 : k + 1);
    s.setTextDatum(top_center);
    s.setTextColor(C_FAINT);
    s.drawString(t, x + bw / 2, top + h + 4);
  }
}

void drawStrips(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  const int top = 2, h = 48, w = 24;
  for (int i = 0; i < NUM_STRIPS; i++) {
    const StripView &v = snap.s[i];
    uint16_t cc = chanColor(v.color);
    int x = STRIP_X0 + i * STRIP_PITCH;
    bool sel = i == snap.sel;
    s.fillSmoothRoundRect(x, top, w, h, 5, v.on == 0 ? C_MUTED : sel ? rgb(30, 38, 50) : C_PANEL);
    if (sel) {
      s.drawRoundRect(x, top, w, h, 5, cc);
      s.drawRoundRect(x + 1, top + 1, w - 2, h - 2, 4, cc);
    }

    char t[4];
    if (i < 16) snprintf(t, sizeof(t), "%d", i + 1);
    else strcpy(t, i == AUX_STRIP ? "AX" : "LR");
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_center);
    s.setTextColor(sel ? C_TEXT : C_DIM);
    s.drawString(t, x + w / 2, top + 4);
    s.fillRect(x + 5, top + 15, w - 10, 2, cc);

    // meter (left) and fader (right)
    const int mTop = top + 20, mH = h - 25;
    s.fillRect(x + 5, mTop, 4, mH, C_GRAPH);
    if (!isnan(v.meter) && v.meter > -60.0f) {
      int mh = (int)lroundf((v.meter + 60.0f) / 60.0f * mH);
      s.fillRect(x + 5, mTop + mH - mh, 4, mh, meterColor(v.meter));
    }
    s.fillRect(x + 15, mTop, 2, mH, C_GRID);
    if (v.haveFader) {
      int fy = mTop + mH - (int)lroundf(v.fader * mH);
      s.fillRect(x + 12, fy - 1, 8, 3, sel ? C_TEXT : rgb(160, 172, 188));
    }
  }
}

// ---------- screens ----------

enum View { V_STATUS, V_MAIN, V_PICKER, V_KEYBOARD, V_SETTINGS, V_SYNC };
enum Screen { SCR_AUTO, SCR_PICKER, SCR_KEYBOARD, SCR_SETTINGS };  // where the user navigated
Screen screen = SCR_AUTO;
View view = V_STATUS;
uint32_t viewHash = 0;    // content of the current full-screen view
char statusLine[64] = "Starting";
bool statusGear = false;
char pendingSsid[33];     // network just tapped, until the main loop starts joining it
uint32_t pendingAt = 0;

void joinFromUi(const char *ssid, const char *pass, bool useSaved) {
  requestJoin(ssid, pass, useSaved);
  strlcpy(pendingSsid, ssid, sizeof(pendingSsid));
  pendingAt = millis();
  screen = SCR_AUTO;
}

void sendAction(UiActionType type, int arg = 0) {
  UiAction a = {type, (int8_t)arg};
  xQueueSend(uiQueue, &a, 0);
}

// ---------- sync screen ----------

// Shown after a mixer is found while its channels load. Progress comes from
// replies actually received, weighted by how many there are.
const uint32_t SYNC_MAX_MS = 6000;   // never hold the console back longer than this
const uint32_t SYNC_READY_MS = 450;  // how long "Ready" shows at 100%
uint32_t syncSeen = 0;               // syncStart the sync screen last ran for
uint32_t syncDoneAt = 0;
bool syncFinished = false;
float syncShown = 0.0f;              // eased progress, 0..1

struct SyncStep { const char *label; float frac; char count[12]; };
SyncStep syncSteps[5];

float syncProgress() {
  const float n = NUM_STRIPS;
  float f = 0;
  f += 18.0f * snap.namesKnown / n;
  f += 18.0f * snap.fadersKnown / n;
  f += 18.0f * snap.mutesKnown / n;
  f += 40.0f * snap.detailFrac;
  f += 6.0f * snap.metersLive;
  return f / 100.0f;
}

void buildSyncSteps() {
  const int n = NUM_STRIPS;
  syncSteps[0] = {"Channel names", (float)snap.namesKnown / n, ""};
  snprintf(syncSteps[0].count, 12, "%d/%d", snap.namesKnown, n);
  syncSteps[1] = {"Faders and mutes", (snap.fadersKnown + snap.mutesKnown) / (2.0f * n), ""};
  snprintf(syncSteps[1].count, 12, "%d/%d", snap.fadersKnown + snap.mutesKnown, 2 * n);
  syncSteps[2] = {"EQ, dynamics, preamp, sends", snap.detailFrac, ""};
  snprintf(syncSteps[2].count, 12, "%d%%", (int)lroundf(snap.detailFrac * 100));
  syncSteps[3] = {"Live meters", snap.metersLive ? 1.0f : 0.0f, ""};
  float data = syncProgress();
  syncSteps[4] = {"Drawing console", data >= 1.0f || syncDoneAt ? 1.0f : 0.0f, ""};
}

// True while the sync screen should show for the current mixer.
bool syncing() {
  if (snap.syncStart != syncSeen) {  // a new mixer (or the same one re-found)
    syncSeen = snap.syncStart;
    syncFinished = false;
    syncDoneAt = 0;
    syncShown = 0.0f;
  }
  if (syncFinished || !snap.syncStart) return false;
  if (!syncDoneAt && (syncProgress() >= 0.999f || millis() - snap.syncStart > SYNC_MAX_MS)) {
    syncDoneAt = millis();
    Serial.printf("[%lu] sync %s in %lu ms: names %d, faders %d, mutes %d, detail %.0f%%, meters %d\n",
                  millis(), syncProgress() >= 0.999f ? "complete" : "timed out",
                  millis() - snap.syncStart, snap.namesKnown, snap.fadersKnown, snap.mutesKnown,
                  snap.detailFrac * 100, snap.metersLive);
  }
  if (syncDoneAt && millis() - syncDoneAt > SYNC_READY_MS && syncShown > 0.99f) syncFinished = true;
  return !syncFinished;
}

const int SC_X = 70, SC_Y = 34, SC_W = 340, SC_H = 252;
const int BAR_X = SC_X + 28, BAR_Y = SC_Y + 92, BAR_W = SC_W - 56, BAR_H = 12;

void drawSyncFrame() {
  tft.fillScreen(C_BG);
  tft.fillSmoothRoundRect(SC_X, SC_Y, SC_W, SC_H, 14, C_PANEL);
  tft.drawRoundRect(SC_X, SC_Y, SC_W, SC_H, 14, C_BORDER);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextColor(C_TEXT);
  tft.drawString("Syncing with mixer", 240, SC_Y + 32);
  char sub[64];
  if (snap.mixerName[0])
    snprintf(sub, sizeof(sub), "%s  -  %s  -  %s", snap.mixerName, snap.mixerModel, snap.mixer);
  else
    snprintf(sub, sizeof(sub), "%s", snap.mixer);
  tft.setFont(&fonts::DejaVu12);
  tft.setTextColor(C_DIM);
  tft.drawString(sub, 240, SC_Y + 60);
}

void drawSyncDynamic(bool full) {
  static int lastFill = -1;
  static uint32_t lastSteps = 0;
  if (full) { lastFill = -1; lastSteps = 0; }

  // Bar and percentage, when the eased value has moved a pixel
  int fill = (int)lroundf(syncShown * BAR_W);
  bool ready = syncDoneAt && syncShown > 0.99f;
  if (fill != lastFill || full) {
    lastFill = fill;
    tft.fillSmoothRoundRect(BAR_X, BAR_Y, BAR_W, BAR_H, BAR_H / 2, C_GRAPH);
    if (fill > 0) tft.fillSmoothRoundRect(BAR_X, BAR_Y, max(fill, BAR_H), BAR_H, BAR_H / 2, ready ? C_GREEN : C_TEAL);
    tft.fillRect(SC_X + 20, BAR_Y + 18, SC_W - 40, 26, C_PANEL);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::DejaVu18);
    char pct[16];
    snprintf(pct, sizeof(pct), "%d%%", (int)lroundf(syncShown * 100));
    tft.setTextColor(ready ? C_GREEN : C_TEXT);
    tft.drawString(ready ? "Ready" : pct, 240, BAR_Y + 31);
  }

  // Checklist, when a step's state or count changes
  int y0 = BAR_Y + 56, active = -1;
  for (int i = 0; i < 5 && active < 0; i++)
    if (syncSteps[i].frac < 0.999f) active = i;
  Hash h;
  h.add(active);
  for (const SyncStep &st : syncSteps) h.add(st.frac >= 0.999f).str(st.count);
  if (h.h != lastSteps) {
    lastSteps = h.h;
    tft.fillRect(SC_X + 16, y0 - 8, SC_W - 32, 5 * 22 + 4, C_PANEL);
    for (int i = 0; i < 5; i++) {
      const SyncStep &st = syncSteps[i];
      int y = y0 + i * 22, ix = SC_X + 36;
      bool done = st.frac >= 0.999f;
      if (done) {
        tft.fillSmoothCircle(ix, y + 3, 7, C_GREEN);
        tft.drawWideLine(ix - 3, y + 3, ix - 1, y + 6, 1.2f, C_BG);
        tft.drawWideLine(ix - 1, y + 6, ix + 4, y, 1.2f, C_BG);
      } else if (i != active) {
        tft.fillSmoothCircle(ix, y + 3, 3, C_GRID);
      }
      tft.setFont(&fonts::DejaVu12);
      tft.setTextDatum(middle_left);
      tft.setTextColor(done || i == active ? C_TEXT : C_FAINT);
      tft.drawString(st.label, ix + 18, y + 3);
      if (st.count[0]) {
        tft.setTextDatum(middle_right);
        tft.setTextColor(done ? C_DIM : C_FAINT);
        tft.drawString(st.count, SC_X + SC_W - 30, y + 3);
      }
    }
  }

  // Spinner on the step in progress
  if (active >= 0) {
    int ix = SC_X + 36, iy = y0 + active * 22 + 3;
    tft.fillRect(ix - 9, iy - 9, 19, 19, C_PANEL);
    tft.drawCircle(ix, iy, 7, C_TEAL);
    float a = (millis() % 900) / 900.0f * 2 * PI;
    tft.fillSmoothCircle(ix + cosf(a) * 4, iy + sinf(a) * 4, 2, C_TEAL);
  }
}

// Decides what the TFT shows: boot/network status, a Wi-Fi or settings
// screen, or the mixer view.
View pickView() {
  statusGear = true;
  if (snap.calibrating) {
    snprintf(statusLine, sizeof(statusLine), "Calibrating fader");
    statusGear = false;
    return V_STATUS;
  }
  if (screen == SCR_KEYBOARD) return V_KEYBOARD;
  if (screen == SCR_SETTINGS) return V_SETTINGS;
  if (screen == SCR_PICKER) return V_PICKER;
  if (pendingAt && snap.net != NET_CONNECTING && millis() - pendingAt < 1500) {
    snprintf(statusLine, sizeof(statusLine), "Joining %s", pendingSsid);
    return V_STATUS;
  }
  pendingAt = 0;
  if (snap.net == NET_CONNECTING) {
    snprintf(statusLine, sizeof(statusLine), "Joining %s", snap.netSsid);
    return V_STATUS;
  }
  if (snap.net == NET_CONNECTED && !snap.online) {
    snprintf(statusLine, sizeof(statusLine), "Reconnecting to %s", snap.netSsid);
    return V_STATUS;
  }
  if (snap.net != NET_CONNECTED) {
    if (snap.net == NET_IDLE && snap.scanning && snap.scanCount == 0) {
      snprintf(statusLine, sizeof(statusLine), "Looking for Wi-Fi networks");
      return V_STATUS;
    }
    return V_PICKER;  // nothing joined: pick a network
  }
  if (!snap.haveMixer) {
    snprintf(statusLine, sizeof(statusLine), "Looking for the mixer on %s", snap.netSsid);
    return V_STATUS;
  }
  if (syncing()) return V_SYNC;
  return V_MAIN;
}

// Rounded button; `accent` fills it.
void button(int x, int y, int w, int h, const char *label, uint16_t accent = 0, bool enabled = true) {
  if (accent) tft.fillSmoothRoundRect(x, y, w, h, 8, accent);
  else {
    tft.fillSmoothRoundRect(x, y, w, h, 8, C_PANEL);
    tft.drawRoundRect(x, y, w, h, 8, C_BORDER);
  }
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextDatum(middle_center);
  tft.setTextColor(accent ? C_BG : enabled ? C_TEXT : C_FAINT);
  tft.drawString(label, x + w / 2, y + h / 2 + 1);
}

bool inside(int px, int py, int x, int y, int w, int h, int pad = 4) {
  return px >= x - pad && px < x + w + pad && py >= y - pad && py < y + h + pad;
}

void signalBars(int x, int yBottom, int rssi, uint16_t color) {
  int bars = rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
  for (int b = 0; b < 4; b++) {
    int h = 5 + b * 4;
    tft.fillRect(x + b * 6, yBottom - h, 4, h, b < bars ? color : C_GRID);
  }
}

void padlock(int x, int y, uint16_t color) {
  tft.drawRoundRect(x + 2, y, 8, 9, 3, color);
  tft.drawRoundRect(x + 3, y + 1, 6, 8, 2, color);
  tft.fillSmoothRoundRect(x, y + 6, 12, 9, 2, color);
}

// ---------- status screen ----------

void drawStatusView(bool full) {
  static uint32_t lastDots = 0;
  static int dots = 0;
  if (full) {
    tft.fillScreen(C_BG);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold24pt7b);
    tft.setTextColor(C_TEXT);
    tft.drawString("XR18", 240, 118);
    tft.setFont(&fonts::DejaVu12);
    tft.setTextColor(C_DIM);
    tft.drawString("FADER CONSOLE", 240, 154);
    tft.setFont(&fonts::DejaVu18);
    tft.setTextColor(C_TEXT);
    tft.drawString(statusLine, 240, 212);
    if (statusGear) drawGear(tft, 448, 30, C_DIM, C_BG);
  }
  if (millis() - lastDots > 300) {  // progress dots
    lastDots = millis();
    dots = (dots + 1) % 4;
    for (int i = 0; i < 3; i++)
      tft.fillSmoothCircle(222 + i * 18, 246, 4, i < dots ? C_TEAL : C_GRID);
  }
}

// ---------- Wi-Fi picker ----------

const int PICK_ROWS = 4, PICK_Y = 80, PICK_ROW_H = 46;
int pickPage = 0;

bool pickerCanGoBack() { return snap.net == NET_CONNECTED && snap.online; }

void drawPicker() {
  tft.fillScreen(C_BG);
  tft.setTextDatum(middle_left);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextColor(C_TEXT);
  tft.drawString("Wi-Fi", 16, 28);
  if (pickerCanGoBack()) button(276, 10, 90, 36, "BACK");
  button(376, 10, 94, 36, snap.scanning ? "SCANNING" : "SCAN", 0, !snap.scanning);

  tft.setFont(&fonts::DejaVu12);
  tft.setTextDatum(middle_left);
  char line[64];
  if (snap.net == NET_FAILED) {
    snprintf(line, sizeof(line), "Couldn't join %s - check the password", snap.netSsid);
    tft.setTextColor(C_AMBER);
  } else if (snap.scanning && snap.scanCount == 0) {
    snprintf(line, sizeof(line), "Looking for networks...");
    tft.setTextColor(C_DIM);
  } else if (snap.scanCount == 0) {
    snprintf(line, sizeof(line), "No networks found - tap SCAN to try again");
    tft.setTextColor(C_DIM);
  } else {
    snprintf(line, sizeof(line), "Tap a network to join it");
    tft.setTextColor(C_DIM);
  }
  tft.drawString(line, 16, 62);

  int pages = max(1, (snap.scanCount + PICK_ROWS - 1) / PICK_ROWS);
  pickPage = constrain(pickPage, 0, pages - 1);
  for (int r = 0; r < PICK_ROWS; r++) {
    int i = pickPage * PICK_ROWS + r;
    if (i >= snap.scanCount) break;
    const ScanEntry &e = snap.scan[i];
    int y = PICK_Y + r * PICK_ROW_H;
    bool current = snap.online && !strcmp(e.ssid, snap.netSsid);
    tft.fillSmoothRoundRect(12, y, 456, 42, 8, current ? rgb(24, 44, 40) : C_PANEL);
    signalBars(26, y + 31, e.rssi, current ? C_TEAL : C_DIM);
    tft.setFont(&fonts::DejaVu18);
    tft.setTextDatum(middle_left);
    tft.setTextColor(C_TEXT);
    tft.drawString(e.ssid, 60, y + 21);
    int x = 450;
    if (!e.open) { padlock(x - 12, y + 13, C_DIM); x -= 22; }
    const char *tag = current ? "CONNECTED" : isSaved(e.ssid) ? "SAVED" : nullptr;
    if (tag) {
      tft.setFont(&fonts::DejaVu9);
      int w = tft.textWidth(tag) + 12;
      tft.fillSmoothRoundRect(x - w, y + 14, w, 15, 7, current ? C_TEAL : C_BORDER);
      tft.setTextDatum(middle_center);
      tft.setTextColor(current ? C_BG : C_TEXT);
      tft.drawString(tag, x - w / 2, y + 22);
    }
  }
  if (pages > 1) {
    button(12, 270, 100, 40, "PREV", 0, pickPage > 0);
    button(368, 270, 100, 40, "NEXT", 0, pickPage < pages - 1);
    char t[16];
    snprintf(t, sizeof(t), "%d / %d", pickPage + 1, pages);
    tft.setFont(&fonts::DejaVu12);
    tft.setTextDatum(middle_center);
    tft.setTextColor(C_DIM);
    tft.drawString(t, 240, 290);
  }
}

uint32_t pickerHash() {
  Hash h;
  h.add(snap.scanSeq).add(snap.scanning).add(snap.scanCount).add(snap.net).add(snap.online)
   .add(pickPage).add(snap.savedCount).str(snap.netSsid).add(pickerCanGoBack());
  return h.h;
}

// ---------- password keyboard ----------

enum KeyKind : uint8_t { K_CHAR, K_SHIFT, K_BACK, K_MODE, K_SPACE, K_SHOW, K_CANCEL, K_JOIN };
struct Key { int16_t x, y, w, h; char ch; KeyKind kind; };
Key keys[56];
int keyCount = 0;
char kbSsid[33], kbText[65];
bool kbShift = false, kbSym = false, kbShow = false;
const int KB_ROW_Y = 86, KB_KEY_H = 42, KB_GAP = 4;

void addKey(int x, int y, int w, char ch, KeyKind kind) {
  if (keyCount < (int)(sizeof(keys) / sizeof(keys[0]))) keys[keyCount++] = {(int16_t)x, (int16_t)y, (int16_t)w, KB_KEY_H, ch, kind};
}

void addCharRow(const char *chars, int y, int x0, int x1) {
  int n = strlen(chars);
  int w = (x1 - x0 - (n - 1) * KB_GAP) / n;
  int used = n * w + (n - 1) * KB_GAP, x = x0 + (x1 - x0 - used) / 2;
  for (int i = 0; i < n; i++, x += w + KB_GAP) addKey(x, y, w, chars[i], K_CHAR);
}

void buildKeys() {
  static const char *const LETTERS[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
  static const char *const SYMBOLS[] = {"1234567890", "!@#$%^&*()", "-_=+[]{}\\|", ";:'\",./?"};
  const char *const *rows = kbSym ? SYMBOLS : LETTERS;
  keyCount = 0;
  int y = KB_ROW_Y;
  addCharRow(rows[0], y, 4, 476);
  y += KB_KEY_H + KB_GAP;
  addCharRow(rows[1], y, 4, 476);
  y += KB_KEY_H + KB_GAP;
  addCharRow(rows[2], y, kbSym ? 4 : 28, kbSym ? 476 : 452);
  y += KB_KEY_H + KB_GAP;
  addKey(4, y, 60, 0, K_SHIFT);
  addCharRow(rows[3], y, 68, 412);
  addKey(416, y, 60, 0, K_BACK);
  y += KB_KEY_H + KB_GAP;
  addKey(4, y, 70, 0, K_MODE);
  addKey(78, y, 160, ' ', K_SPACE);
  addKey(242, y, 64, 0, K_SHOW);
  addKey(310, y, 76, 0, K_CANCEL);
  addKey(390, y, 86, 0, K_JOIN);
}

char keyChar(const Key &k) {
  return (!kbSym && kbShift && k.ch >= 'a' && k.ch <= 'z') ? k.ch - 32 : k.ch;
}

void drawKbField() {
  tft.fillRect(0, 26, 480, 54, C_BG);
  tft.fillSmoothRoundRect(12, 30, 456, 44, 8, C_PANEL);
  tft.drawRoundRect(12, 30, 456, 44, 8, C_TEAL);
  char shown[65];
  size_t n = strlen(kbText);
  if (kbShow) strlcpy(shown, kbText, sizeof(shown));
  else { memset(shown, '*', n); shown[n] = 0; }
  tft.setFont(&fonts::DejaVu18);
  tft.setTextDatum(middle_left);
  tft.setTextColor(n ? C_TEXT : C_FAINT);
  const char *t = n ? shown : "password";
  // Keep the end of long passwords visible
  while (*t && tft.textWidth(t) > 420) t++;
  tft.drawString(t, 26, 52);
  int cx = 26 + (n ? tft.textWidth(t) : 0) + 2;
  tft.fillRect(cx, 40, 2, 24, C_TEAL);
}

void drawKeyboard() {
  tft.fillScreen(C_BG);
  tft.setFont(&fonts::DejaVu12);
  tft.setTextDatum(middle_left);
  tft.setTextColor(C_DIM);
  char t[64];
  snprintf(t, sizeof(t), "Password for %s", kbSsid);
  tft.drawString(t, 16, 14);
  drawKbField();
  buildKeys();
  for (int i = 0; i < keyCount; i++) {
    const Key &k = keys[i];
    uint16_t fill = C_PANEL, text = C_TEXT;
    char label[8] = {0};
    switch (k.kind) {
      case K_CHAR:   label[0] = keyChar(k); break;
      case K_SPACE:  strcpy(label, "space"); text = C_DIM; break;
      case K_SHIFT:  strcpy(label, kbSym ? "" : "SHIFT"); if (kbShift && !kbSym) { fill = C_TEXT; text = C_BG; } break;
      case K_BACK:   strcpy(label, "DEL"); break;
      case K_MODE:   strcpy(label, kbSym ? "ABC" : "?123"); break;
      case K_SHOW:   strcpy(label, kbShow ? "HIDE" : "SHOW"); break;
      case K_CANCEL: strcpy(label, "CANCEL"); text = C_DIM; break;
      case K_JOIN:   strcpy(label, "JOIN"); fill = C_TEAL; text = C_BG; break;
    }
    tft.fillSmoothRoundRect(k.x, k.y, k.w, k.h, 6, fill);
    tft.setFont(k.kind == K_CHAR ? &fonts::DejaVu18 : &fonts::DejaVu9);
    tft.setTextDatum(middle_center);
    tft.setTextColor(text);
    tft.drawString(label, k.x + k.w / 2, k.y + k.h / 2 + 1);
  }
}

uint32_t keyboardHash() { return Hash().add(kbShift).add(kbSym).add(kbShow).str(kbSsid).h; }

void openKeyboard(const char *ssid) {
  strlcpy(kbSsid, ssid, sizeof(kbSsid));
  kbText[0] = 0;
  kbShift = kbSym = kbShow = false;
  screen = SCR_KEYBOARD;
}

void keyboardTap(int x, int y) {
  const Key *hit = nullptr;
  for (int i = 0; i < keyCount && !hit; i++)
    if (inside(x, y, keys[i].x, keys[i].y, keys[i].w, keys[i].h, 2)) hit = &keys[i];
  if (!hit) return;
  size_t n = strlen(kbText);
  switch (hit->kind) {
    case K_CHAR:
    case K_SPACE:
      if (n < sizeof(kbText) - 1) {
        kbText[n] = hit->kind == K_SPACE ? ' ' : keyChar(*hit);
        kbText[n + 1] = 0;
      }
      if (kbShift) kbShift = false;  // one capital, like a phone
      break;
    case K_BACK:   if (n) kbText[n - 1] = 0; break;
    case K_SHIFT:  if (!kbSym) kbShift = !kbShift; break;
    case K_MODE:   kbSym = !kbSym; kbShift = false; break;
    case K_SHOW:   kbShow = !kbShow; break;
    case K_CANCEL: screen = SCR_PICKER; break;
    case K_JOIN:
      joinFromUi(kbSsid, kbText, false);
      break;
  }
}

// ---------- settings ----------

void drawSettings() {
  tft.fillScreen(C_BG);
  tft.setTextDatum(middle_left);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setTextColor(C_TEXT);
  tft.drawString("Settings", 16, 28);
  button(376, 10, 94, 36, "CLOSE");

  tft.fillSmoothRoundRect(12, 58, 456, 108, 10, C_PANEL);
  tft.setFont(&fonts::DejaVu9);
  tft.setTextColor(C_DIM);
  tft.drawString("WI-FI", 28, 76);
  tft.drawString("CONTROLLER", 28, 128);
  tft.drawString("MIXER", 248, 128);
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(snap.online ? C_TEXT : C_AMBER);
  tft.drawString(snap.online ? snap.netSsid : "Not connected", 28, 98);
  if (snap.online) signalBars(430, 106, snap.rssi, C_TEAL);
  tft.setFont(&fonts::DejaVu12);
  tft.setTextColor(C_TEXT);
  tft.drawString(snap.online ? snap.ip : "-", 28, 148);
  tft.setTextColor(snap.haveMixer ? C_TEXT : C_DIM);
  tft.drawString(snap.haveMixer ? snap.mixer : "searching...", 248, 148);

  button(12, 182, 222, 52, "CHANGE WI-FI");
  button(246, 182, 222, 52, "RECALIBRATE TOUCH");
  tft.setFont(&fonts::DejaVu9);
  tft.setTextDatum(middle_center);
  tft.setTextColor(C_FAINT);
  tft.drawString("Saved networks are joined automatically when they're in range", 240, 262);
}

uint32_t settingsHash() {
  Hash h;
  h.add(snap.online).str(snap.netSsid).str(snap.ip).str(snap.mixer).add(snap.haveMixer)
   .add((int8_t)(snap.rssi / 6));
  return h.h;
}

// ---------- touch ----------

bool readTouchScreen(int &x, int &y);
void calibrateScreenTouch();
void saveTouchCalibration();

const bool SHOW_TAPS = false;  // draw a dot where each tap registered (for checking calibration)
int16_t tapX = -1, tapY = -1;
uint32_t tapAt = 0;

void mainTap(int x, int y, const char *&what) {
  const int pad = 6;  // forgiving edges around buttons
  if (inside(x, y, GEAR_X - 22, 0, 44, 52, 0)) {
    screen = SCR_SETTINGS;
    what = "settings";
  } else if (y >= BTN_Y - pad && y < BTN_Y + BTN_H + pad && x >= MUTE_X - pad && x < MUTE_X + BTN_W + pad / 2) {
    sendAction(UI_MUTE);
    pressedButton = 0;
    pressedAt = millis();
    what = "MUTE";
  } else if (y >= BTN_Y - pad && y < BTN_Y + BTN_H + pad && x >= SOLO_X - pad / 2 && x < SOLO_X + BTN_W + pad) {
    sendAction(UI_SOLO);
    pressedButton = 1;
    pressedAt = millis();
    what = "SOLO";
  } else if (y >= pStrips.y - pad) {
    // Nearest strip centre, so taps in the gaps still land somewhere sensible.
    int i = (int)lroundf((float)(x - STRIP_X0 - 12) / STRIP_PITCH);
    i = constrain(i, 0, NUM_STRIPS - 1);
    sendAction(UI_SELECT, i);
    what = snap.s[i].label;
  }
}

void pickerTap(int x, int y) {
  if (pickerCanGoBack() && inside(x, y, 276, 10, 90, 36)) { screen = SCR_AUTO; return; }
  if (inside(x, y, 376, 10, 94, 36)) { sendAction(UI_SCAN); return; }
  int pages = max(1, (snap.scanCount + PICK_ROWS - 1) / PICK_ROWS);
  if (pages > 1 && inside(x, y, 12, 270, 100, 40)) { pickPage = max(0, pickPage - 1); return; }
  if (pages > 1 && inside(x, y, 368, 270, 100, 40)) { pickPage = min(pages - 1, pickPage + 1); return; }
  for (int r = 0; r < PICK_ROWS; r++) {
    int i = pickPage * PICK_ROWS + r;
    if (i >= snap.scanCount || !inside(x, y, 12, PICK_Y + r * PICK_ROW_H, 456, 42, 1)) continue;
    const ScanEntry &e = snap.scan[i];
    bool failedHere = snap.net == NET_FAILED && !strcmp(e.ssid, snap.netSsid);
    if (e.open) joinFromUi(e.ssid, "", false);
    else if (isSaved(e.ssid) && !failedHere) joinFromUi(e.ssid, "", true);
    else openKeyboard(e.ssid);  // new network, or its saved password failed
    return;
  }
}

void settingsTap(int x, int y) {
  if (inside(x, y, 376, 10, 94, 36)) screen = SCR_AUTO;
  else if (inside(x, y, 12, 182, 222, 52)) { screen = SCR_PICKER; pickPage = 0; sendAction(UI_SCAN); }
  else if (inside(x, y, 246, 182, 222, 52)) {
    calibrateScreenTouch();
    saveTouchCalibration();
    viewHash = 0;  // redraw settings
  }
}

void hitTest(int x, int y) {
  const char *what = "nothing";
  switch (view) {
    case V_MAIN:     mainTap(x, y, what); break;
    case V_PICKER:   pickerTap(x, y); what = "picker"; break;
    case V_KEYBOARD: keyboardTap(x, y); what = "keyboard"; break;
    case V_SETTINGS: settingsTap(x, y); what = "settings"; break;
    case V_STATUS:
      if (statusGear && inside(x, y, 420, 4, 56, 52, 0)) { screen = SCR_SETTINGS; what = "settings"; }
      break;
  }
  Serial.printf("[%lu] tap %d,%d -> %s\n", millis(), x, y, what);
  tapX = x;
  tapY = y;
  tapAt = millis();
}

// A resistive panel's first reading of a press is unreliable (the finger
// hasn't settled), so ignore it and act on the average of the next few.
void handleTouch() {
  static int samples = 0, sumX = 0, sumY = 0, firstX = 0, firstY = 0;
  static bool fired = false;
  int tx, ty;
  bool down = readTouchScreen(tx, ty);
  if (down) {
    if (samples == 0) { firstX = tx; firstY = ty; }
    else { sumX += tx; sumY += ty; }
    samples++;
    if (!fired && samples >= 4) {
      fired = true;
      hitTest(sumX / (samples - 1), sumY / (samples - 1));
    }
  } else {
    if (!fired && samples > 0) {  // a very quick tap: use what we have
      if (samples > 1) hitTest(sumX / (samples - 1), sumY / (samples - 1));
      else hitTest(firstX, firstY);
    }
    samples = sumX = sumY = 0;
    fired = false;
  }
}

void drawTapMarker() {
  static bool shown = false;
  if (!SHOW_TAPS) return;
  if (tapAt && millis() - tapAt < 700) {
    tft.fillSmoothCircle(tapX, tapY, 5, C_TEXT);
    tft.drawCircle(tapX, tapY, 8, C_RED);
    shown = true;
  } else if (shown) {
    shown = false;
    for (Panel *p : PANELS) p->hash = 0;  // redraw over the marker
  }
}

// Touch calibration: raw XPT2046 readings -> screen pixels, as an affine fit
// (sx = a*rx + b*ry + c, sy = d*rx + e*ry + f) over five targets set in from
// the edges, where a resistive panel reads more accurately than the corners.
// Saved to flash; runs on first boot, or if the screen is held at the splash.
float touchAffine[6] = {0, 0, 0, 0, 0, 0};
bool touchCalibrated = false;

bool readTouchScreen(int &x, int &y) {
  lgfx::touch_point_t tp;
  if (!touchCalibrated || tft.getTouchRaw(&tp, 1) == 0) return false;
  x = (int)lroundf(touchAffine[0] * tp.x + touchAffine[1] * tp.y + touchAffine[2]);
  y = (int)lroundf(touchAffine[3] * tp.x + touchAffine[4] * tp.y + touchAffine[5]);
  return true;
}

// Waits for a press, averages the settled raw readings, waits for release.
void sampleRawTouch(float &rx, float &ry) {
  lgfx::touch_point_t tp;
  for (;;) {
    while (tft.getTouchRaw(&tp, 1) == 0) delay(5);
    delay(60);  // let the finger settle
    float sx = 0, sy = 0;
    int n = 0;
    while (n < 24 && tft.getTouchRaw(&tp, 1) > 0) {
      sx += tp.x;
      sy += tp.y;
      n++;
      delay(8);
    }
    while (tft.getTouchRaw(&tp, 1) > 0) delay(5);  // wait for release
    delay(150);
    if (n >= 6) {  // a proper press, not a brush
      rx = sx / n;
      ry = sy / n;
      return;
    }
  }
}

// Least-squares fit of out = a*rx + b*ry + c over n points.
bool fitAxis(const float *rx, const float *ry, const float *out, int n, float *abc) {
  double m[3][4] = {};
  for (int i = 0; i < n; i++) {
    double v[3] = {rx[i], ry[i], 1.0};
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) m[r][c] += v[r] * v[c];
      m[r][3] += v[r] * out[i];
    }
  }
  for (int k = 0; k < 3; k++) {  // Gauss-Jordan
    int piv = k;
    for (int r = k + 1; r < 3; r++) if (fabs(m[r][k]) > fabs(m[piv][k])) piv = r;
    if (fabs(m[piv][k]) < 1e-9) return false;
    for (int c = 0; c < 4; c++) std::swap(m[k][c], m[piv][c]);
    for (int r = 0; r < 3; r++) {
      if (r == k) continue;
      double f = m[r][k] / m[k][k];
      for (int c = k; c < 4; c++) m[r][c] -= f * m[k][c];
    }
  }
  for (int k = 0; k < 3; k++) abc[k] = (float)(m[k][3] / m[k][k]);
  return true;
}

void drawTarget(int x, int y, uint16_t color) {
  tft.drawCircle(x, y, 12, color);
  tft.drawFastHLine(x - 18, y, 37, color);
  tft.drawFastVLine(x, y - 18, 37, color);
  tft.fillSmoothCircle(x, y, 3, color);
}

void calibrateScreenTouch() {
  static const int TX[5] = {40, 440, 440, 40, 240};
  static const int TY[5] = {40, 40, 280, 280, 160};
  float rx[5], ry[5], fx[5], fy[5];
  for (int i = 0; i < 5; i++) { fx[i] = TX[i]; fy[i] = TY[i]; }

  for (;;) {
    tft.fillScreen(C_BG);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::DejaVu18);
    tft.setTextColor(C_TEXT);
    tft.drawString("Touch calibration", 240, 110);
    tft.setFont(&fonts::DejaVu12);
    tft.setTextColor(C_DIM);
    tft.drawString("Press and hold the centre of each target", 240, 200);
    tft.drawString("with a fingernail or stylus, then let go", 240, 218);

    for (int i = 0; i < 5; i++) {
      drawTarget(TX[i], TY[i], C_TEAL);
      sampleRawTouch(rx[i], ry[i]);
      drawTarget(TX[i], TY[i], C_BG);
      tft.fillSmoothCircle(TX[i], TY[i], 4, C_GREEN);
      Serial.printf("cal target %d,%d raw %.0f,%.0f\n", TX[i], TY[i], rx[i], ry[i]);
    }

    bool ok = fitAxis(rx, ry, fx, 5, touchAffine) && fitAxis(rx, ry, fy, 5, touchAffine + 3);
    float worst = 0;
    for (int i = 0; ok && i < 5; i++) {
      float ex = touchAffine[0] * rx[i] + touchAffine[1] * ry[i] + touchAffine[2] - TX[i];
      float ey = touchAffine[3] * rx[i] + touchAffine[4] * ry[i] + touchAffine[5] - TY[i];
      worst = max(worst, sqrtf(ex * ex + ey * ey));
    }
    Serial.printf("Touch calibration fit: worst error %.1f px\n", worst);
    if (ok && worst < 12.0f) break;

    tft.fillScreen(C_BG);
    tft.setFont(&fonts::DejaVu18);
    tft.setTextColor(C_AMBER);
    tft.drawString("That didn't line up - let's try again", 240, 160);
    delay(1500);
  }
}

void saveTouchCalibration() {
  Preferences prefs;
  prefs.begin("xrpad", false);
  prefs.putBytes("taff", touchAffine, sizeof(touchAffine));
  prefs.end();
  Serial.println("Touchscreen calibrated");
}

void setupTouch() {
  Preferences prefs;
  prefs.begin("xrpad", false);
  bool have = prefs.getBytes("taff", touchAffine, sizeof(touchAffine)) == sizeof(touchAffine);

  uint32_t until = millis() + 1500;
  bool held = false;
  lgfx::touch_point_t tp;
  while (millis() < until && !held) {
    held = tft.getTouchRaw(&tp, 1) > 0;
    delay(20);
  }
  if (held) while (tft.getTouchRaw(&tp, 1) > 0) delay(5);  // let go first

  prefs.end();
  if (!have || held) {
    calibrateScreenTouch();
    saveTouchCalibration();
  }
  touchCalibrated = true;
}

// ---------- task ----------

void drawMain() {
  const StripView &v = snap.s[snap.sel];
  const Detail &d = snap.d;
  bool flash = millis() - pressedAt < PRESS_FLASH_MS;

  Hash hh;
  hh.add(snap.sel).str(v.label).str(v.name).add(v.color).add(v.on).add(d.solo).add(d.lr).add(flash);
  Hash hl;
  hl.add(snap.sel).q(v.fader, 1023).add(v.haveFader).q(v.meter, 2).add(v.color)
    .add(snap.pickedUp).add(snap.faderTouched).q(snap.pickedUp ? 0 : snap.faderPos, 200);
  Hash he;
  he.add(snap.sel).add(d.color).add(d.eqOn).add(d.hpOn).q(d.hpf, 1000);
  for (const EqBand &b : d.eq) he.add(b.type).q(b.f, 1000).q(b.g, 1000).q(b.q, 1000);
  Hash hi;
  hi.add(snap.sel).add(snap.meterSeq).add(d.gateOn).q(d.gateThr, 1000).add(d.dynOn)
    .q(d.dynThr, 1000).add(d.dynRatio);
  Hash hp;
  hp.add(snap.sel).add(d.color).q(d.gain, 1000).add(d.phantom).add(d.invert).add(d.lr).q(d.pan, 1000);
  Hash hs;
  hs.add(snap.sel);
  for (float x : d.sends) hs.q(x, 1000);
  Hash ho;
  ho.add(snap.sel);
  for (const StripView &sv : snap.s) ho.q(sv.fader, 400).q(sv.meter, 1).add(sv.on).add(sv.color).add(sv.haveFader);

  struct { Panel *p; uint32_t h; void (*draw)(LGFX_Sprite &); } jobs[] = {
    {&pHeader, hh.h, drawHeader}, {&pLevel, hl.h, drawLevel}, {&pEq, he.h, drawEq},
    {&pHistory, hi.h, drawHistory}, {&pPreamp, hp.h, drawPreamp}, {&pSends, hs.h, drawSends},
    {&pStrips, ho.h, drawStrips},
  };
  for (auto &j : jobs) {
    if (j.p->hash == j.h) continue;
    j.p->hash = j.h;
    j.draw(j.p->spr);
    j.p->spr.pushSprite(&tft, j.p->x, j.p->y);
  }
}

void uiTask(void *) {
  static uint32_t fieldHash = 0;
  for (;;) {
    uint32_t start = millis();
    takeSnapshot();
    View v = pickView();
    bool changed = v != view;
    view = v;
    if (changed) {
      viewHash = 0;
      for (Panel *p : PANELS) p->hash = 0;
      if (view == V_MAIN) tft.fillScreen(C_BG);
    }

    uint32_t h;
    switch (view) {
      case V_STATUS:
        h = Hash().str(statusLine).add(statusGear).h;
        drawStatusView(changed || h != viewHash);
        viewHash = h;
        break;
      case V_PICKER:
        h = pickerHash();
        if (changed || h != viewHash) { viewHash = h; drawPicker(); }
        break;
      case V_KEYBOARD:
        h = keyboardHash();
        if (changed || h != viewHash) {
          viewHash = h;
          drawKeyboard();
          fieldHash = Hash().str(kbText).add(kbShow).h;
        } else if ((h = Hash().str(kbText).add(kbShow).h) != fieldHash) {
          fieldHash = h;
          drawKbField();  // typing only redraws the text box
        }
        break;
      case V_SETTINGS:
        h = settingsHash();
        if (changed || h != viewHash) { viewHash = h; drawSettings(); }
        break;
      case V_SYNC: {
        float target = syncDoneAt ? 1.0f : syncProgress();
        syncShown += (target - syncShown) * 0.18f;  // ease towards the real progress
        if (target - syncShown < 0.002f) syncShown = target;
        buildSyncSteps();
        if (changed) drawSyncFrame();
        drawSyncDynamic(changed);
        break;
      }
      case V_MAIN:
        drawMain();
        break;
    }

    handleTouch();
    if (view == V_MAIN) drawTapMarker();
    uint32_t took = millis() - start;
    vTaskDelay(pdMS_TO_TICKS(took < FRAME_MS ? FRAME_MS - took : 1));
  }
}

}  // namespace ui

// Called from setup(): splash, touchscreen calibration, then the UI task.
void uiBegin() {
  using namespace ui;
  tft.init();
  tft.setRotation(ROTATION);
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  statusGear = false;
  drawStatusView(true);
  setupTouch();
  view = V_STATUS;
  viewHash = 0;  // the calibration screen may have replaced the splash

  for (Panel *p : PANELS) {
    p->spr.setColorDepth(16);
    p->spr.setPsram(true);
    p->spr.createSprite(p->w, p->h);
    p->hash = 0;
  }
  xTaskCreatePinnedToCore(uiTask, "ui", 8192, nullptr, 1, nullptr, 0);
}
