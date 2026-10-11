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
  int mode;  // 0 calibrating fader, 1 joining Wi-Fi, 2 finding mixer, 3 running
  int sel;
  StripView s[NUM_STRIPS];
  Detail d;
  int8_t hist[HIST_LEN];
  uint32_t meterSeq;
  bool pickedUp, faderTouched;
  float faderPos;  // where the physical fader is
};

Snap snap;

void takeSnapshot() {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  snap.mode = bootStage == 0 ? 0 : WiFi.status() != WL_CONNECTED ? 1 : !haveMixer ? 2 : 3;
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
  xSemaphoreGive(stateLock);
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
  if (s.textWidth(name) > MUTE_X - 36) s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextColor(C_TEXT);
  s.setTextDatum(bottom_left);
  s.drawString(name, 24, 48);

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

// ---------- status screens ----------

int statusShown = -1;  // which status screen is on the TFT, 3 = the mixer UI

// Mode 3 (running) just clears the screen once so the panels draw fresh.
void drawStatusScreen(int mode) {
  static uint32_t lastDots = 0;
  static int dots = 0;
  if (mode == 3) {
    if (statusShown != 3) {
      statusShown = 3;
      tft.fillScreen(C_BG);
      for (Panel *p : PANELS) p->hash = 0;
    }
    return;
  }
  if (mode != statusShown) {
    statusShown = mode;
    tft.fillScreen(C_BG);
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold24pt7b);
    tft.setTextColor(C_TEXT);
    tft.drawString("XR18", 240, 118);
    tft.setFont(&fonts::DejaVu12);
    tft.setTextColor(C_DIM);
    tft.drawString("FADER CONSOLE", 240, 154);
    char line[48];
    if (mode == 0)      snprintf(line, sizeof(line), "Calibrating fader");
    else if (mode == 1) snprintf(line, sizeof(line), "Joining %s", WIFI_SSID);
    else                snprintf(line, sizeof(line), "Looking for the mixer");
    tft.setFont(&fonts::DejaVu18);
    tft.setTextColor(C_TEXT);
    tft.drawString(line, 240, 212);
  }
  if (millis() - lastDots > 300) {  // simple progress dots
    lastDots = millis();
    dots = (dots + 1) % 4;
    for (int i = 0; i < 3; i++)
      tft.fillSmoothCircle(222 + i * 18, 246, 4, i < dots ? C_TEAL : C_GRID);
  }
}

// ---------- touch ----------

void sendAction(UiActionType type, int arg = 0) {
  UiAction a = {type, (int8_t)arg};
  xQueueSend(uiQueue, &a, 0);
}

bool readTouchScreen(int &x, int &y);

const bool SHOW_TAPS = true;  // draw a dot where each tap registered (for checking calibration)
int16_t tapX = -1, tapY = -1;
uint32_t tapAt = 0;

void hitTest(int x, int y) {
  const int pad = 6;  // forgiving edges around buttons
  const char *what = "nothing";
  if (y >= BTN_Y - pad && y < BTN_Y + BTN_H + pad && x >= MUTE_X - pad && x < MUTE_X + BTN_W + pad / 2) {
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

  if (!have || held) {
    calibrateScreenTouch();
    prefs.putBytes("taff", touchAffine, sizeof(touchAffine));
    Serial.println("Touchscreen calibrated");
  }
  touchCalibrated = true;
  prefs.end();
}

// ---------- task ----------

void uiTask(void *) {
  for (;;) {
    uint32_t start = millis();
    takeSnapshot();
    drawStatusScreen(snap.mode);
    if (snap.mode == 3) {
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
    handleTouch();
    if (snap.mode == 3) drawTapMarker();
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
  drawStatusScreen(0);
  setupTouch();
  statusShown = -1;  // the calibration screen may have replaced the splash

  for (Panel *p : PANELS) {
    p->spr.setColorDepth(16);
    p->spr.setPsram(true);
    p->spr.createSprite(p->w, p->h);
    p->hash = 0;
  }
  xTaskCreatePinnedToCore(uiTask, "ui", 8192, nullptr, 1, nullptr, 0);
}
