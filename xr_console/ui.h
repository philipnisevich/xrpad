// 4.0" 480x320 ST7796 TFT with XPT2046 touch: the selected channel in detail
// plus an overview of every strip. Runs in its own task on core 0 so drawing
// never holds up the fader, motor or network. Each panel is drawn into an
// off-screen sprite and pushed only when what it shows has changed.
//
// Included from xr_console.ino after the state it reads (strips, details,
// meters, network), which it copies under stateLock once per frame. Taps
// either act directly (toggles, pages) or ask the main loop to point the
// encoder at a parameter.

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

  uint8_t editId, editIdx;  // what the encoder is adjusting

  // Home screen: every strip's details, buses/FX/DCAs, mixer-wide settings
  Detail dAll[NUM_STRIPS];
  Other oth[NUM_OTHERS];
  MixerInfo mi;
  float meters40[40];

  // 8-fader board
  uint8_t bank;
  bool fbConnected, fbHasTouch;
  int fbFaders;
  uint16_t fbTouch;
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
  snap.editId = editId;
  snap.editIdx = editIdx;
  memcpy(snap.dAll, details, sizeof(details));
  memcpy(snap.oth, others, sizeof(others));
  snap.mi = mixInfo;
  memcpy(snap.meters40, allMeters, sizeof(allMeters));
  snap.bank = bank;
  snap.fbConnected = fbConnected;
  snap.fbHasTouch = fbHasTouch;
  snap.fbFaders = fbFaders;
  snap.fbTouch = fbTouch;
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

// ---------- layout and touch zones ----------

// Every tappable thing registers a zone while it's drawn, so hitboxes always
// match what's on screen. Zones are in screen coordinates.
enum ZoneAct : uint8_t { Z_EDIT, Z_TOGGLE, Z_PAGE, Z_BACK, Z_STRIP, Z_MUTE, Z_SOLO, Z_GEAR, Z_BAND, Z_STOP,
                         Z_MUTE_ANY, Z_MUTEGRP, Z_OPEN_CH, Z_TAB, Z_HOME, Z_SETINS };
struct Zone { int16_t x, y, w, h; uint8_t act, id, idx; };
const int MAX_ZONES = 40;

struct Panel {
  int16_t x, y, w, h;
  LGFX_Sprite spr;
  uint32_t hash;
  Zone zones[MAX_ZONES];
  int nz;
};

// Channel overview
Panel pHeader  = {0, 0, 480, 52};
Panel pLevel   = {8, 58, 104, 204};
Panel pEq      = {118, 58, 236, 124};
Panel pDyn     = {118, 188, 236, 74};
Panel pPreamp  = {360, 58, 112, 124};
Panel pSends   = {360, 188, 112, 74};
Panel pStrips  = {0, 268, 480, 52};
// Full pages: a title bar and up to two areas, sized per page
Panel pPgTitle = {0, 0, 480, 48};
Panel pPgA     = {0, 48, 480, 220};
Panel pPgB     = {0, 48, 0, 0};
// Home screen
Panel pHomeTabs = {0, 0, 480, 40};
Panel pHomeBody = {0, 40, 480, 280};
Panel *const MAIN_PANELS[] = {&pHeader, &pLevel, &pEq, &pDyn, &pPreamp, &pSends, &pStrips};
Panel *const PAGE_PANELS[] = {&pPgTitle, &pPgA, &pPgB, &pStrips};
Panel *const HOME_PANELS[] = {&pHomeTabs, &pHomeBody};
Panel *const PANELS[] = {&pHeader, &pLevel, &pEq, &pDyn, &pPreamp, &pSends, &pStrips,
                         &pPgTitle, &pPgA, &pPgB, &pHomeTabs, &pHomeBody};

const int16_t MUTE_X = 248, SOLO_X = 338, BTN_Y = 8, BTN_W = 82, BTN_H = 36;
const int16_t GEAR_X = 452, GEAR_Y = 26;  // settings gear, far right of the header
const int16_t STRIP_X0 = 6, STRIP_PITCH = 26;

enum Page : uint8_t { PG_NONE, PG_EQ, PG_GATE, PG_COMP, PG_SENDS, PG_PREAMP, PG_INSERT };
Page page = PG_NONE;
int pageBand = 0;            // EQ page: 0-3 bands, 4 = low cut
bool channelOpen = false;    // false = home screen, true = the selected channel
int homeTab = 0;

int pressedButton = -1;      // 0 mute, 1 solo, for the tap flash
uint32_t pressedAt = 0;

Panel *zonePanel = nullptr;  // panel being drawn

void beginZones(Panel &p) { p.nz = 0; zonePanel = &p; }

void zone(int x, int y, int w, int h, uint8_t act, uint8_t id = 0, uint8_t idx = 0) {
  if (!zonePanel || zonePanel->nz >= MAX_ZONES) return;
  zonePanel->zones[zonePanel->nz++] = {(int16_t)(zonePanel->x + x), (int16_t)(zonePanel->y + y),
                                       (int16_t)w, (int16_t)h, act, id, idx};
}

bool editing(uint8_t id, uint8_t idx = 0) { return snap.editId == id && snap.editIdx == idx; }

// ---------- parameter values and text ----------

float gateRangeDb(float x) { return 3.0f + x * 57.0f; }
float attackMs(float x)    { return x * 120.0f; }
float holdMs(float x)      { return 0.02f * powf(100000.0f, x); }
float releaseMs(float x)   { return 5.0f * powf(800.0f, x); }
float kneeVal(float x)     { return x * 5.0f; }
float makeupDb(float x)    { return x * 24.0f; }
float mixPct(float x)      { return x * 100.0f; }

const char *const EQ_TYPES[6] = {"LOW CUT", "LOW SHELF", "PEQ", "VINTAGE", "HIGH SHELF", "HIGH CUT"};

float uiParam(uint8_t id, uint8_t idx) {
  const Detail &d = snap.d;
  const StripView &v = snap.s[snap.sel];
  auto i = [](int8_t x) { return x < 0 ? NAN : (float)x; };
  switch (id) {
    case P_FADER:      return v.haveFader ? v.fader : NAN;
    case P_PAN:        return d.pan;
    case P_GAIN:       return d.gain;
    case P_PHANTOM:    return i(d.phantom);
    case P_INVERT:     return i(d.invert);
    case P_LR:         return i(d.lr);
    case P_HPON:       return i(d.hpOn);
    case P_HPF:        return d.hpf;
    case P_EQON:       return i(d.eqOn);
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
    case P_OTHER:      return idx == OTHER_PHONES ? snap.mi.phonesLevel
                            : idx < NUM_OTHERS ? snap.oth[idx].fader : NAN;
  }
  return NAN;
}

void freqText(char *out, size_t n, float hz) {
  if (hz < 1000.0f) snprintf(out, n, "%.0f Hz", hz);
  else if (hz < 10000.0f) snprintf(out, n, "%.2f kHz", hz / 1000.0f);
  else snprintf(out, n, "%.1f kHz", hz / 1000.0f);
}

void msText(char *out, size_t n, float ms) {
  if (ms < 10.0f) snprintf(out, n, "%.1f ms", ms);
  else if (ms < 1000.0f) snprintf(out, n, "%.0f ms", ms);
  else snprintf(out, n, "%.2f s", ms / 1000.0f);
}

void paramText(uint8_t id, uint8_t idx, char *out, size_t n) {
  float v = uiParam(id, idx);
  if (isnan(v)) { snprintf(out, n, "--"); return; }
  switch (id) {
    case P_FADER: case P_SEND: case P_OTHER:
      if (v <= 0.0f) snprintf(out, n, "-inf"); else snprintf(out, n, "%+.1f dB", faderToDb(v));
      break;
    case P_PAN: {
      int p = (int)lroundf(v * 200.0f - 100.0f);
      if (p == 0) snprintf(out, n, "C"); else snprintf(out, n, "%c%d", p < 0 ? 'L' : 'R', abs(p));
      break;
    }
    case P_GAIN:       snprintf(out, n, "%+.1f dB", preampDb(v)); break;
    case P_HPF:        freqText(out, n, hpfFreq(v)); break;
    case P_EQ_TYPE:    snprintf(out, n, "%s", EQ_TYPES[constrain((int)v, 0, 5)]); break;
    case P_EQ_F:       freqText(out, n, eqFreq(v)); break;
    case P_EQ_G:       snprintf(out, n, "%+.1f dB", eqGain(v)); break;
    case P_EQ_Q:       snprintf(out, n, "%.1f", eqQ(v)); break;
    case P_GATE_THR:   snprintf(out, n, "%.1f dB", gateDb(v)); break;
    case P_GATE_RANGE: snprintf(out, n, "%.0f dB", gateRangeDb(v)); break;
    case P_GATE_ATT: case P_DYN_ATT: msText(out, n, attackMs(v)); break;
    case P_GATE_HOLD:  msText(out, n, holdMs(v)); break;
    case P_GATE_REL: case P_DYN_REL: msText(out, n, releaseMs(v)); break;
    case P_DYN_THR:    snprintf(out, n, "%.1f dB", dynDb(v)); break;
    case P_DYN_RATIO:  snprintf(out, n, "%g:1", RATIOS[constrain((int)v, 0, 11)]); break;
    case P_DYN_KNEE:   snprintf(out, n, "%.0f", kneeVal(v)); break;
    case P_DYN_MGAIN:  snprintf(out, n, "%+.1f dB", makeupDb(v)); break;
    case P_DYN_MIX:    snprintf(out, n, "%.0f%%", mixPct(v)); break;
    default:           snprintf(out, n, "%s", v > 0.5f ? "ON" : "OFF"); break;
  }
}

void paramLabel(uint8_t id, uint8_t idx, char *out, size_t n) {
  if (id >= P_EQ_TYPE && id <= P_EQ_Q) snprintf(out, n, "EQ %d %s", idx + 1, PARAMS[id].name);
  else if (id >= P_GATE_THR && id <= P_GATE_REL) snprintf(out, n, "GATE %s", PARAMS[id].name);
  else if (id >= P_DYN_THR && id <= P_DYN_MIX) snprintf(out, n, "COMP %s", PARAMS[id].name);
  else if (id == P_SEND) snprintf(out, n, idx < 6 ? "BUS %d SEND" : "FX %d SEND", idx < 6 ? idx + 1 : idx - 5);
  else if (id == P_OTHER) {
    if (idx == OTHER_PHONES) snprintf(out, n, "HEADPHONES");
    else if (idx < OTHER_FXSEND && idx >= OTHER_BUS) snprintf(out, n, "BUS %d", idx - OTHER_BUS + 1);
    else if (idx < OTHER_BUS) snprintf(out, n, "FX %d RETURN", idx + 1);
    else if (idx < OTHER_DCA) snprintf(out, n, "FX %d SEND", idx - OTHER_FXSEND + 1);
    else snprintf(out, n, "DCA %d", idx - OTHER_DCA + 1);
  }
  else snprintf(out, n, "%s", PARAMS[id].name);
}

// ---------- drawing helpers ----------

// Rounded panel with a small caption in the top-left corner.
void panelBase(LGFX_Sprite &s, const char *title) {
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString(title, 10, 7);
}

// Caption that opens a page, with a little chevron to show it's tappable.
void pageCaption(LGFX_Sprite &s, const char *title, Page pg, int w = 80) {
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString(title, 10, 7);
  int tx = 10 + s.textWidth(title) + 4;
  s.drawLine(tx, 8, tx + 3, 11, C_DIM);
  s.drawLine(tx + 3, 11, tx, 14, C_DIM);
  zone(0, 0, w, 22, Z_PAGE, pg);
}

void selectedOutline(LGFX_Sprite &s, int x, int y, int w, int h, int r = 6) {
  s.drawRoundRect(x, y, w, h, r, C_TEAL);
  s.drawRoundRect(x + 1, y + 1, w - 2, h - 2, r > 1 ? r - 1 : r, C_TEAL);
}

// Small rounded on/off label that toggles a parameter when tapped.
int pill(LGFX_Sprite &s, int xRight, int y, const char *text, uint16_t color, bool on,
         uint8_t toggleId = P_NONE) {
  s.setFont(&fonts::DejaVu9);
  int w = s.textWidth(text) + 12, x = xRight - w;
  if (on) {
    s.fillSmoothRoundRect(x, y, w, 15, 7, color);
    s.setTextColor(C_BG);
  } else {
    s.drawRoundRect(x, y, w, 15, 7, C_BORDER);
    s.setTextColor(C_FAINT);
  }
  s.setTextDatum(middle_center);
  s.drawString(text, x + w / 2, y + 8);
  if (toggleId) zone(x - 2, y - 4, w + 4, 23, Z_TOGGLE, toggleId);
  return x - 4;  // where the next pill to the left should end
}

void dashedHLine(LGFX_Sprite &s, int x0, int x1, int y, uint16_t color) {
  for (int x = x0; x < x1; x += 6) s.drawFastHLine(x, y, min(3, x1 - x), color);
}

void dashedVLine(LGFX_Sprite &s, int x, int y0, int y1, uint16_t color) {
  for (int y = y0; y < y1; y += 6) s.drawFastVLine(x, y, min(3, y1 - y), color);
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

// A labelled value box: tap to edit with the knob, or to toggle.
void tile(LGFX_Sprite &s, int x, int y, int w, int h, uint8_t id, uint8_t idx = 0) {
  char label[24], value[20];
  paramLabel(id, idx, label, sizeof(label));
  if (id >= P_EQ_TYPE && id <= P_DYN_MIX) {  // drop the "EQ 1 " / "GATE " prefix inside pages
    const char *sp = strrchr(label, ' ');
    if (id >= P_EQ_TYPE && id <= P_EQ_Q && sp) memmove(label, sp + 1, strlen(sp));
    else if ((id >= P_GATE_THR && id <= P_DYN_MIX) && sp) memmove(label, strchr(label, ' ') + 1, strlen(strchr(label, ' ')));
  }
  paramText(id, idx, value, sizeof(value));
  bool toggle = PARAMS[id].kind == 'b';
  float v = uiParam(id, idx);
  bool on = toggle && v > 0.5f;
  bool sel = editing(id, idx);
  uint16_t accent = id == P_PHANTOM ? C_RED : id == P_INVERT ? C_AMBER : C_TEAL;

  s.fillSmoothRoundRect(x, y, w, h, 8, on ? mix565(C_PANEL, accent, 0.35f) : sel ? rgb(26, 44, 48) : C_PANEL);
  if (sel) selectedOutline(s, x, y, w, h, 8);
  else s.drawRoundRect(x, y, w, h, 8, on ? accent : C_BORDER);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(sel ? C_TEAL : C_DIM);
  s.drawString(label, x + 10, y + 8);
  s.setFont(h >= 56 ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold9pt7b);
  if (s.textWidth(value) > w - 16) s.setFont(&fonts::FreeSansBold9pt7b);
  s.setTextDatum(bottom_left);
  s.setTextColor(isnan(v) ? C_FAINT : on ? C_TEXT : toggle ? C_DIM : C_TEXT);
  s.drawString(value, x + 10, y + h - 7);
  zone(x, y, w, h, toggle ? Z_TOGGLE : Z_EDIT, id, idx);
}

// ---------- graphs ----------

// EQ response: the four bands (when on) plus the low cut. Handles register
// zones that select a band (pages) or edit its gain (overview).
void eqGraph(LGFX_Sprite &s, int gx, int gy, int gw, int gh, bool big) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  s.fillSmoothRoundRect(gx, gy, gw, gh, 6, C_GRAPH);
  auto xFor = [&](float hz) { return gx + (int)lroundf(log10f(hz / 20.0f) / 3.0f * (gw - 1)); };
  auto yFor = [&](float db) { return gy + gh / 2 - (int)lroundf(constrain(db, -15.0f, 15.0f) * (gh / 2 - 4) / 15.0f); };

  static const int MINOR[] = {50, 200, 500, 2000, 5000};
  for (int hz : MINOR) s.drawFastVLine(xFor(hz), gy + 2, gh - 4, C_GRID);
  static const int MAJOR[] = {100, 1000, 10000};
  static const char *const MAJOR_T[] = {"100", "1k", "10k"};
  s.setFont(&fonts::DejaVu9);
  for (int i = 0; i < 3; i++) {
    s.drawFastVLine(xFor(MAJOR[i]), gy + 2, gh - 4, C_BORDER);
    s.setTextDatum(bottom_center);
    s.setTextColor(C_FAINT);
    s.drawString(MAJOR_T[i], xFor(MAJOR[i]), gy + gh - 2);
  }
  for (int db : {-12, -6, 6, 12}) s.drawFastHLine(gx + 2, yFor(db), gw - 4, C_GRID);
  int zeroY = yFor(0);
  s.drawFastHLine(gx + 2, zeroY, gw - 4, C_GRID0);
  s.setTextDatum(middle_left);
  s.drawString("+12", gx + 4, yFor(12));
  s.drawString("-12", gx + 4, yFor(-12));

  bool eqKnown = d.eqOn >= 0, hpKnown = d.hpOn >= 0 && !isnan(d.hpf);
  bool bandsKnown = eqKnown;
  for (const EqBand &b : d.eq)
    if (b.type < 0 || isnan(b.f) || isnan(b.g) || isnan(b.q)) bandsKnown = false;
  if (!bandsKnown && !hpKnown) {
    s.setTextDatum(middle_center);
    s.setTextColor(C_FAINT);
    s.drawString(snap.sel == MAIN_STRIP ? "" : "reading EQ...", gx + gw / 2, gy + gh / 2 - 12);
    return;
  }

  bool eqActive = bandsKnown && d.eqOn == 1, hpActive = hpKnown && d.hpOn == 1;
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
    if (x > 0) s.drawWideLine(gx + x - 1, prevY, gx + x, y, big ? 1.4f : 1.0f, lineColor);
    prevY = y;
  }

  if (big && hpKnown) {  // low cut handle on the page
    int x = constrain(xFor(hpfFreq(d.hpf)), gx + 9, gx + gw - 10);
    bool sel = pageBand == 4;
    s.fillSmoothCircle(x, zeroY, sel ? 10 : 8, d.hpOn == 1 ? C_TEAL : C_FAINT);
    if (sel) s.drawCircle(x, zeroY, 13, C_TEXT);
    s.setTextColor(C_BG);
    s.setTextDatum(middle_center);
    s.drawString("LC", x, zeroY + 1);
    zone(x - 14, zeroY - 14, 28, 28, Z_BAND, 0, 4);
  }
  if (bandsKnown) {
    int r = big ? 9 : 7;
    for (int i = 0; i < 4; i++) {
      const EqBand &b = d.eq[i];
      bool cut = b.type == 0 || b.type == 5;
      int x = constrain(xFor(eqFreq(b.f)), gx + r, gx + gw - r - 1);
      int y = cut ? zeroY : yFor(eqGain(b.g));
      bool sel = big && pageBand == i;
      uint16_t c = d.eqOn == 1 ? BAND_COLORS[i] : C_FAINT;
      s.fillSmoothCircle(x, y, sel ? r + 1 : r, c);
      if (sel) s.drawCircle(x, y, r + 4, C_TEXT);
      s.setTextColor(C_BG);
      s.setTextDatum(middle_center);
      char t[2] = {(char)('1' + i), 0};
      s.drawString(t, x, y + 1);
      if (big) zone(x - 15, y - 15, 30, 30, Z_BAND, 0, i);
    }
  }
}

// Output level against input level for the gate or compressor, with the
// channel's live input as a dot riding the curve.
float dynOut(bool comp, float in) {
  const Detail &d = snap.d;
  if (!comp) {
    float thr = gateDb(d.gateThr), range = isnan(d.gateRange) ? 40.0f : gateRangeDb(d.gateRange);
    return in >= thr ? in : in - range;
  }
  float thr = dynDb(d.dynThr);
  float ratio = d.dynRatio >= 0 && d.dynRatio < 12 ? RATIOS[d.dynRatio] : 3.0f;
  float knee = isnan(d.dynKnee) ? 0.0f : kneeVal(d.dynKnee) * 5.0f;  // knee width in dB
  float makeup = isnan(d.dynMgain) ? 0.0f : makeupDb(d.dynMgain);
  float over = in - thr, out;
  if (knee > 0 && fabsf(over) <= knee / 2) out = in + (1.0f / ratio - 1.0f) * powf(over + knee / 2, 2) / (2 * knee);
  else if (over > 0) out = thr + over / ratio;
  else out = in;
  return out + makeup;
}

void dynGraph(LGFX_Sprite &s, int gx, int gy, int gw, int gh, bool comp, bool big) {
  const Detail &d = snap.d;
  bool known = comp ? !isnan(d.dynThr) : !isnan(d.gateThr);
  bool on = comp ? d.dynOn == 1 : d.gateOn == 1;
  uint16_t accent = comp ? C_ORANGE : C_RED;
  s.fillSmoothRoundRect(gx, gy, gw, gh, big ? 6 : 4, C_GRAPH);
  const float lo = -60.0f;
  auto xFor = [&](float db) { return gx + (int)lroundf((constrain(db, lo, 0.0f) - lo) / -lo * (gw - 1)); };
  auto yFor = [&](float db) { return gy + gh - 1 - (int)lroundf((constrain(db, lo, 0.0f) - lo) / -lo * (gh - 1)); };
  if (big) {
    for (int db = -48; db < 0; db += 12) {
      s.drawFastVLine(xFor(db), gy + 2, gh - 4, C_GRID);
      s.drawFastHLine(gx + 2, yFor(db), gw - 4, C_GRID);
    }
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.setTextDatum(bottom_left);
    s.drawString("IN", gx + gw - 20, gy + gh - 3);
    s.setTextDatum(top_left);
    s.drawString("OUT", gx + 4, gy + 4);
  }
  // 1:1 reference
  for (int i = 0; i < gw; i += 4) s.drawPixel(gx + i, gy + gh - 1 - i * (gh - 1) / (gw - 1), C_GRID0);
  if (!known) return;

  float thr = comp ? dynDb(d.dynThr) : gateDb(d.gateThr);
  if (thr > lo) dashedVLine(s, xFor(thr), gy + 2, gy + gh - 2, on ? accent : C_FAINT);
  uint16_t curve = on ? accent : C_FAINT;
  int prevY = 0;
  for (int x = 0; x < gw; x++) {
    float in = lo + (float)x / (gw - 1) * -lo;
    int y = yFor(dynOut(comp, in));  // its shape even when off, just greyed
    if (x > 0) s.drawWideLine(gx + x - 1, prevY, gx + x, y, big ? 1.5f : 1.0f, curve);
    prevY = y;
  }
  float m = snap.s[snap.sel].meter;
  if (!isnan(m) && m > lo) {
    float out = on ? dynOut(comp, m) : m;
    int x = xFor(m), y = yFor(out);
    s.fillSmoothCircle(x, y, big ? 6 : 3, mix565(C_GRAPH, C_TEXT, 0.35f));
    s.fillSmoothCircle(x, y, big ? 4 : 2, C_TEXT);
  }
}

// ---------- overview panels ----------

void drawHeader(LGFX_Sprite &s) {
  const StripView &v = snap.s[snap.sel];
  const Detail &d = snap.d;
  uint16_t cc = chanColor(v.color);
  bool flash = millis() - pressedAt < PRESS_FLASH_MS;
  s.fillScreen(C_BG);

  // home button
  s.fillSmoothRoundRect(6, 8, 38, 36, 8, C_PANEL);
  s.drawRoundRect(6, 8, 38, 36, 8, C_BORDER);
  s.fillTriangle(14, 26, 25, 15, 36, 26, C_DIM);
  s.fillRect(17, 25, 16, 11, C_DIM);
  s.fillRect(23, 29, 4, 7, C_PANEL);
  zone(0, 0, 50, 52, Z_HOME);

  s.fillSmoothRoundRect(52, 8, 6, 36, 3, cc);
  s.setTextDatum(top_left);
  s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_DIM);
  s.drawString(v.label, 66, 4);

  // What the knob is adjusting: a two-line card centred in the space
  // between the name and the gear. Tap it to stop editing.
  char label[24] = "", value[20] = "";
  int cardW = 0;
  if (snap.editId != P_NONE) {
    paramLabel(snap.editId, snap.editIdx, label, sizeof(label));
    paramText(snap.editId, snap.editIdx, value, sizeof(value));
    s.setFont(&fonts::DejaVu9);
    int lw = s.textWidth(label);
    s.setFont(&fonts::FreeSansBold9pt7b);
    cardW = max(lw, (int)s.textWidth(value)) + 20;
  }
  const int spaceEnd = MUTE_X - 10;
  const char *name = v.name[0] ? v.name : v.label;
  const lgfx::IFont *fonts[] = {&fonts::FreeSansBold18pt7b, &fonts::FreeSansBold12pt7b, &fonts::FreeSansBold9pt7b};
  int nameW = 0;
  for (const lgfx::IFont *f : fonts) {  // biggest font that leaves room for the card
    s.setFont(f);
    nameW = s.textWidth(name);
    if (64 + nameW + (cardW ? cardW + 12 : 0) <= spaceEnd) break;
  }
  s.setTextColor(C_TEXT);
  s.setTextDatum(baseline_left);
  s.drawString(name, 64, 44);

  if (cardW) {
    int free0 = 64 + nameW + 8, free1 = spaceEnd;
    int x = max(free0, (free0 + free1) / 2 - cardW / 2);
    s.fillSmoothRoundRect(x, 6, cardW, 40, 8, rgb(18, 48, 50));
    s.drawRoundRect(x, 6, cardW, 40, 8, C_TEAL);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_center);
    s.setTextColor(C_TEAL);
    s.drawString(label, x + cardW / 2, 11);
    s.setFont(&fonts::FreeSansBold9pt7b);
    s.setTextDatum(bottom_center);
    s.setTextColor(C_TEXT);
    s.drawString(value, x + cardW / 2, 42);
    zone(x - 2, 2, cardW + 4, 48, Z_STOP);
  }

  drawGear(s, GEAR_X, GEAR_Y, C_DIM, C_BG);
  zone(GEAR_X - 24, 0, 52, 52, Z_GEAR);

  bool muted = v.on == 0;
  uint16_t mFill = muted ? C_RED : C_PANEL;
  if (flash && pressedButton == 0) mFill = mix565(mFill, C_TEXT, 0.3f);
  s.fillSmoothRoundRect(MUTE_X, BTN_Y, BTN_W, BTN_H, 8, mFill);
  if (!muted) s.drawRoundRect(MUTE_X, BTN_Y, BTN_W, BTN_H, 8, C_BORDER);
  s.setFont(&fonts::FreeSansBold9pt7b);
  s.setTextDatum(middle_center);
  s.setTextColor(muted ? C_TEXT : v.on < 0 ? C_FAINT : C_DIM);
  s.drawString(muted ? "MUTED" : "MUTE", MUTE_X + BTN_W / 2, BTN_Y + BTN_H / 2 + 1);
  zone(MUTE_X - 4, 2, BTN_W + 6, BTN_H + 12, Z_MUTE);

  bool soloOk = snap.sel != MAIN_STRIP, solo = d.solo == 1;
  uint16_t sFill = solo ? C_AMBER : C_PANEL;
  if (flash && pressedButton == 1 && soloOk) sFill = mix565(sFill, C_TEXT, 0.3f);
  s.fillSmoothRoundRect(SOLO_X, BTN_Y, BTN_W, BTN_H, 8, sFill);
  if (!solo) s.drawRoundRect(SOLO_X, BTN_Y, BTN_W, BTN_H, 8, C_BORDER);
  s.setTextColor(solo ? C_BG : soloOk ? C_DIM : C_GRID);
  s.drawString("SOLO", SOLO_X + BTN_W / 2, BTN_Y + BTN_H / 2 + 1);
  zone(SOLO_X - 2, 2, BTN_W + 6, BTN_H + 12, Z_SOLO);
}

void drawLevel(LGFX_Sprite &s) {
  const StripView &v = snap.s[snap.sel];
  uint16_t cc = chanColor(v.color);
  panelBase(s, "LEVEL");
  if (editing(P_FADER)) selectedOutline(s, 0, 0, s.width(), s.height(), 8);
  const int top = 24, bottom = 168, h = bottom - top;

  // Segmented input meter, -60..0 dBFS
  const int segs = 36, segH = h / segs;
  for (int k = 0; k < segs; k++) {
    float segDb = -60.0f + 60.0f * (k + 1) / segs;
    uint16_t c = meterColor(segDb);
    bool lit = !isnan(v.meter) && v.meter >= segDb - 60.0f / segs;
    s.fillRect(10, bottom - (k + 1) * segH + 1, 12, segH - 1, lit ? c : mix565(C_PANEL, c, 0.15f));
  }

  static const int MARKS[] = {10, 5, 0, -5, -10, -20, -30, -50};
  s.setFont(&fonts::DejaVu9);
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

  if (!snap.pickedUp && snap.faderPos >= 0.0f && v.haveFader) {
    int gy = bottom - (int)lroundf(snap.faderPos * h);
    s.drawRoundRect(52, gy - 7, 30, 14, 3, C_FAINT);
  }
  if (v.haveFader) {
    int y = bottom - (int)lroundf(v.fader * h);
    s.fillSmoothRoundRect(52, y - 7, 30, 14, 3, snap.faderTouched ? C_TEXT : rgb(176, 188, 202));
    s.fillRect(55, y - 1, 24, 2, cc);
    if (snap.faderTouched) s.drawRoundRect(51, y - 8, 32, 16, 4, cc);
  }

  char t[12];
  if (v.haveFader) faderText(t, sizeof(t), v.fader);
  else strcpy(t, "--");
  s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextDatum(middle_center);
  s.setTextColor(editing(P_FADER) ? C_TEAL : C_TEXT);
  s.drawString(t, 52, 184);
  s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_FAINT);
  s.drawString("dB", 52, 198);
  zone(0, 0, s.width(), s.height(), Z_EDIT, P_FADER);
}

void drawEqPanel(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  pageCaption(s, "EQ", PG_EQ);
  int right = s.width() - 8;
  if (d.eqOn >= 0) right = pill(s, right, 4, d.eqOn ? "ON" : "OFF", C_GREEN, d.eqOn, P_EQON);
  if (d.hpOn >= 0) pill(s, right, 4, "LOW CUT", C_TEAL, d.hpOn, P_HPON);
  zone(0, 22, s.width(), s.height() - 22, Z_PAGE, PG_EQ);  // anywhere else opens the page
  eqGraph(s, 8, 24, s.width() - 16, s.height() - 30, false);
}

void dynHalf(LGFX_Sprite &s, int x0, bool comp) {
  const Detail &d = snap.d;
  int8_t on = comp ? d.dynOn : d.gateOn;
  uint16_t accent = comp ? C_ORANGE : C_RED;
  const int w = 118;
  // caption + toggle
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  const char *title = comp ? "COMP" : "GATE";
  s.drawString(title, x0 + 10, 7);
  int tx = x0 + 10 + s.textWidth(title) + 4;
  s.drawLine(tx, 8, tx + 3, 11, C_DIM);
  s.drawLine(tx + 3, 11, tx, 14, C_DIM);
  zone(x0, 0, w, s.height(), Z_PAGE, comp ? PG_COMP : PG_GATE);
  if (on >= 0) pill(s, x0 + w - 6, 4, on ? "ON" : "OFF", accent, on == 1, comp ? P_DYN_ON : P_GATE_ON);

  uint8_t thrId = comp ? P_DYN_THR : P_GATE_THR;
  dynGraph(s, x0 + 8, 24, 44, 44, comp, false);

  char t[20];
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_FAINT);
  s.drawString("THR", x0 + 60, 26);
  paramText(thrId, 0, t, sizeof(t));
  s.setTextColor(C_TEXT);
  s.setFont(&fonts::DejaVu12);
  s.drawString(t, x0 + 60, 37);
  s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_FAINT);
  s.drawString(comp ? "RATIO" : "RANGE", x0 + 60, 52);
  paramText(comp ? P_DYN_RATIO : P_GATE_RANGE, 0, t, sizeof(t));
  s.setTextColor(C_DIM);
  s.drawString(t, x0 + 60, 62);
}

void drawDynPanel(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  dynHalf(s, 0, false);
  s.drawFastVLine(118, 8, s.height() - 16, C_BORDER);
  dynHalf(s, 118, true);
}

void drawPreamp(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  pageCaption(s, "PREAMP", PG_PREAMP, 70);

  const int cx = 36, cy = 58;
  if (editing(P_GAIN)) s.drawCircle(cx, cy, 31, C_TEAL), s.drawCircle(cx, cy, 32, C_TEAL);
  arc(s, cx, cy, 21, 27, 135.0f, 405.0f, C_GRAPH);
  s.setTextDatum(middle_center);
  if (!isnan(d.gain)) {
    arc(s, cx, cy, 21, 27, 135.0f, 135.0f + 270.0f * constrain(d.gain, 0.0f, 1.0f), cc);
    char t[8];
    snprintf(t, sizeof(t), "%+.0f", preampDb(d.gain));
    s.setFont(&fonts::FreeSansBold9pt7b);
    s.setTextColor(editing(P_GAIN) ? C_TEAL : C_TEXT);
    s.drawString(t, cx, cy - 1);
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.drawString("dB", cx, cy + 13);
    zone(cx - 32, cy - 32, 64, 64, Z_EDIT, P_GAIN);
  } else {
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.drawString(snap.sel == MAIN_STRIP ? "n/a" : "--", cx, cy);
  }

  auto badge = [&](int y, const char *text, int8_t state, uint16_t color, uint8_t id) {
    if (state < 0) return;
    if (state) s.fillSmoothRoundRect(70, y, 34, 17, 4, color);
    else s.drawRoundRect(70, y, 34, 17, 4, C_BORDER);
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(state ? C_BG : C_FAINT);
    s.setTextDatum(middle_center);
    s.drawString(text, 87, y + 9);
    zone(66, y - 2, 44, 21, Z_TOGGLE, id);
  };
  badge(20, "48V", d.phantom, C_RED, P_PHANTOM);
  badge(39, "INV", d.invert, C_AMBER, P_INVERT);
  badge(58, "LR", d.lr, rgb(96, 165, 250), P_LR);
  if (snap.sel < 16) {  // insert effect: opens the insert page
    bool on = d.fxslot > 0 && d.insOn == 1;
    char t[6] = "FX";
    if (d.fxslot > 0) snprintf(t, sizeof(t), "FX%d%c", (d.fxslot + 1) / 2, d.fxslot % 2 ? 'A' : 'B');
    if (on) s.fillSmoothRoundRect(70, 77, 34, 17, 4, C_PURPLE);
    else s.drawRoundRect(70, 77, 34, 17, 4, C_BORDER);
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(on ? C_BG : C_FAINT);
    s.setTextDatum(middle_center);
    s.drawString(t, 87, 86);
    zone(66, 75, 44, 21, Z_PAGE, PG_INSERT);
  }

  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(middle_left);
  s.setTextColor(editing(P_PAN) ? C_TEAL : C_DIM);
  s.drawString("PAN", 10, 102);
  if (!isnan(d.pan)) {
    char t[8];
    paramText(P_PAN, 0, t, sizeof(t));
    s.setTextDatum(middle_right);
    s.setTextColor(C_TEXT);
    s.drawString(t, 102, 102);
  }
  const int bx = 10, bw = 92, by = 114;
  if (editing(P_PAN)) selectedOutline(s, 4, 96, 104, 26, 6);
  s.fillSmoothRoundRect(bx, by, bw, 5, 2, C_GRAPH);
  s.drawFastVLine(bx + bw / 2, by - 3, 11, C_GRID0);
  if (!isnan(d.pan)) {
    int px = bx + (int)lroundf(d.pan * bw), mid = bx + bw / 2;
    s.fillRect(min(px, mid), by + 1, abs(px - mid), 3, cc);
    s.fillSmoothCircle(px, by + 2, 5, C_TEXT);
  }
  zone(2, 97, 108, 27, Z_EDIT, P_PAN);
}

void drawSends(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(0, 0, s.width(), s.height(), 8, C_PANEL);
  pageCaption(s, "SENDS", PG_SENDS, 70);
  const int top = 22, h = 34, bw = 6;
  for (int k = 0; k < 10; k++) {
    bool fx = k >= 6;
    int x = 10 + k * 9 + (fx ? 6 : 0);
    s.fillSmoothRoundRect(x, top, bw, h, 2, C_GRAPH);
    s.drawFastHLine(x - 1, top + h - (int)lroundf(0.75f * h), bw + 2, C_GRID0);
    if (!isnan(d.sends[k])) {
      int fh = (int)lroundf(constrain(d.sends[k], 0.0f, 1.0f) * h);
      if (fh > 0) s.fillSmoothRoundRect(x, top + h - fh, bw, fh, 2, fx ? C_PURPLE : C_TEAL);
    }
    char t[3];
    snprintf(t, sizeof(t), "%d", fx ? k - 5 : k + 1);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_center);
    s.setTextColor(C_FAINT);
    s.drawString(t, x + bw / 2, top + h + 4);
  }
  zone(0, 0, s.width(), s.height(), Z_PAGE, PG_SENDS);
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
    zone(x - 1, 0, STRIP_PITCH, 52, Z_STRIP, 0, i);
  }
}

// ---------- full pages ----------

const char *const PAGE_TITLES[] = {"", "EQ", "GATE", "COMPRESSOR", "SENDS", "PREAMP", "INSERT FX"};

void setPanelGeometry(Panel &p, int x, int y, int w, int h) {
  p.x = x; p.y = y;
  if (p.w != w || p.h != h || (w && !p.spr.getBuffer())) {
    p.w = w; p.h = h;
    p.spr.deleteSprite();
    if (w && h) p.spr.createSprite(w, h);
  }
  p.hash = 0;
  p.nz = 0;
}

// Splits the page area (y 48-268) into A (graph/dial) and B (controls).
void layoutPage(Page pg) {
  switch (pg) {
    case PG_EQ:     setPanelGeometry(pPgA, 0, 48, 320, 220); setPanelGeometry(pPgB, 320, 48, 160, 220); break;
    case PG_GATE:
    case PG_COMP:   setPanelGeometry(pPgA, 0, 48, 236, 220); setPanelGeometry(pPgB, 236, 48, 244, 220); break;
    case PG_SENDS:
    case PG_INSERT: setPanelGeometry(pPgA, 0, 48, 480, 220); setPanelGeometry(pPgB, 0, 48, 0, 0); break;
    case PG_PREAMP: setPanelGeometry(pPgA, 0, 48, 224, 220); setPanelGeometry(pPgB, 224, 48, 256, 220); break;
    default: break;
  }
  pPgTitle.hash = 0;
}

void drawPageTitle(LGFX_Sprite &s) {
  const StripView &v = snap.s[snap.sel];
  s.fillScreen(C_BG);
  // back
  s.fillSmoothRoundRect(8, 7, 96, 34, 8, C_PANEL);
  s.drawRoundRect(8, 7, 96, 34, 8, C_BORDER);
  s.drawWideLine(22, 24, 29, 17, 1.6f, C_TEXT);
  s.drawWideLine(22, 24, 29, 31, 1.6f, C_TEXT);
  s.setFont(&fonts::FreeSansBold9pt7b);
  s.setTextDatum(middle_center);
  s.setTextColor(C_TEXT);
  s.drawString("BACK", 66, 25);
  zone(0, 0, 110, 48, Z_BACK);

  s.fillSmoothRoundRect(114, 10, 5, 28, 2, chanColor(v.color));
  s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextDatum(middle_left);
  s.drawString(PAGE_TITLES[page], 128, 25);
  int x = 128 + s.textWidth(PAGE_TITLES[page]) + 12;
  s.setFont(&fonts::DejaVu12);
  s.setTextColor(C_DIM);
  char t[32];
  snprintf(t, sizeof(t), "%s  %s", v.label, v.name);
  s.drawString(t, x, 26);

  uint8_t onId = page == PG_EQ ? P_EQON : page == PG_GATE ? P_GATE_ON : page == PG_COMP ? P_DYN_ON
                : page == PG_INSERT && snap.sel < 16 ? P_INS_ON : P_NONE;
  if (onId) {
    float on = uiParam(onId, 0);
    if (!isnan(on)) {
      uint16_t accent = page == PG_COMP ? C_ORANGE : page == PG_GATE ? C_RED : page == PG_INSERT ? C_PURPLE : C_GREEN;
      bool o = on > 0.5f;
      s.fillSmoothRoundRect(388, 7, 84, 34, 8, o ? accent : C_PANEL);
      if (!o) s.drawRoundRect(388, 7, 84, 34, 8, C_BORDER);
      s.setFont(&fonts::FreeSansBold9pt7b);
      s.setTextDatum(middle_center);
      s.setTextColor(o ? C_BG : C_DIM);
      s.drawString(o ? "ON" : "OFF", 430, 25);
      zone(384, 0, 96, 48, Z_TOGGLE, onId);
    }
  }
}

// EQ: big curve and band buttons (A), the selected band's settings (B).
void drawEqPageA(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  eqGraph(s, 8, 4, 304, 158, true);
  static const char *const LABELS[] = {"1", "2", "3", "4", "LOW CUT"};
  const int bx[5] = {8, 58, 108, 158, 216}, bw[5] = {44, 44, 44, 44, 96};
  for (int i = 0; i < 5; i++) {
    bool sel = pageBand == i;
    uint16_t c = i < 4 ? BAND_COLORS[i] : C_TEAL;
    s.fillSmoothRoundRect(bx[i], 170, bw[i], 44, 8, sel ? c : C_PANEL);
    if (!sel) s.drawRoundRect(bx[i], 170, bw[i], 44, 8, C_BORDER);
    s.setFont(i < 4 ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold9pt7b);
    s.setTextDatum(middle_center);
    s.setTextColor(sel ? C_BG : c);
    s.drawString(LABELS[i], bx[i] + bw[i] / 2, 193);
    zone(bx[i] - 2, 166, bw[i] + 4, 52, Z_BAND, 0, i);
  }
}

void drawEqPageB(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  if (pageBand == 4) {
    tile(s, 0, 4, 152, 64, P_HPON);
    tile(s, 0, 74, 152, 64, P_HPF);
  } else {
    tile(s, 0, 4, 152, 49, P_EQ_TYPE, pageBand);
    tile(s, 0, 57, 152, 49, P_EQ_F, pageBand);
    tile(s, 0, 110, 152, 49, P_EQ_G, pageBand);
    tile(s, 0, 163, 152, 51, P_EQ_Q, pageBand);
  }
}

void drawDynPageA(LGFX_Sprite &s) {
  bool comp = page == PG_COMP;
  s.fillScreen(C_BG);
  dynGraph(s, 8, 4, 220, 210, comp, true);
  zone(8, 4, 220, 210, Z_EDIT, comp ? P_DYN_THR : P_GATE_THR);
}

void drawDynPageB(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  if (page == PG_GATE) {
    static const uint8_t IDS[] = {P_GATE_THR, P_GATE_RANGE, P_GATE_ATT, P_GATE_HOLD, P_GATE_REL};
    for (int i = 0; i < 5; i++) tile(s, (i % 2) * 120, 4 + (i / 2) * 72, 116, 66, IDS[i]);
  } else {
    static const uint8_t IDS[] = {P_DYN_THR, P_DYN_RATIO, P_DYN_KNEE, P_DYN_MGAIN,
                                  P_DYN_ATT, P_DYN_REL, P_DYN_MIX};
    for (int i = 0; i < 7; i++) tile(s, (i % 2) * 120, 4 + (i / 2) * 53, 116, 49, IDS[i]);
  }
  // Live level readout in the spare slot
  float m = snap.s[snap.sel].meter;
  int x = 120, y = page == PG_GATE ? 148 : 163, h = page == PG_GATE ? 66 : 49;
  s.fillSmoothRoundRect(x, y, 116, h, 8, C_GRAPH);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString(page == PG_GATE ? "INPUT" : "REDUCTION", x + 10, y + 8);
  char t[16];
  if (isnan(m) || m <= -60.0f) strcpy(t, "--");
  else if (page == PG_GATE) snprintf(t, sizeof(t), "%.0f dB", m);
  else {
    float gr = (snap.d.dynOn == 1 ? dynOut(true, m) - (isnan(snap.d.dynMgain) ? 0 : makeupDb(snap.d.dynMgain)) : m) - m;
    snprintf(t, sizeof(t), "%.1f dB", gr);
  }
  s.setFont(h >= 56 ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold9pt7b);
  s.setTextDatum(bottom_left);
  s.setTextColor(C_TEXT);
  s.drawString(t, x + 10, y + h - 7);
}

void drawSendsPage(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  s.fillScreen(C_BG);
  const int top = 26, h = 150, w = 40, pitch = 46, x0 = 12;
  for (int k = 0; k < 10; k++) {
    bool fx = k >= 6, sel = editing(P_SEND, k);
    int x = x0 + k * pitch + (fx ? 4 : 0);
    uint16_t accent = fx ? C_PURPLE : C_TEAL;
    s.fillSmoothRoundRect(x, 0, w, 214, 8, sel ? rgb(26, 44, 48) : C_PANEL);
    if (sel) selectedOutline(s, x, 0, w, 214, 8);
    char t[8];
    snprintf(t, sizeof(t), fx ? "FX %d" : "BUS %d", fx ? k - 5 : k + 1);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_center);
    s.setTextColor(sel ? C_TEAL : C_DIM);
    s.drawString(t, x + w / 2, 8);
    int cx = x + w / 2;
    s.fillSmoothRoundRect(cx - 3, top, 6, h, 3, C_GRAPH);
    int unity = top + h - (int)lroundf(0.75f * h);
    s.drawFastHLine(cx - 10, unity, 20, C_GRID0);
    float v = d.sends[k];
    if (!isnan(v)) {
      int fh = (int)lroundf(constrain(v, 0.0f, 1.0f) * h);
      if (fh > 0) s.fillSmoothRoundRect(cx - 3, top + h - fh, 6, fh, 3, accent);
      int y = top + h - fh;
      s.fillSmoothRoundRect(cx - 13, y - 6, 26, 12, 3, sel ? C_TEXT : rgb(176, 188, 202));
      s.fillRect(cx - 9, y - 1, 18, 2, accent);
    }
    paramText(P_SEND, k, t, sizeof(t));
    if (strstr(t, " dB")) *strstr(t, " dB") = 0;
    s.setFont(&fonts::DejaVu12);
    s.setTextDatum(middle_center);
    s.setTextColor(sel ? C_TEXT : C_DIM);
    s.drawString(t, cx, 196);
    zone(x - 2, 0, w + 4, 214, Z_EDIT, P_SEND, k);
  }
}

void drawPreampPageA(LGFX_Sprite &s) {
  const Detail &d = snap.d;
  uint16_t cc = chanColor(d.color);
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(8, 0, 208, 214, 10, editing(P_GAIN) ? rgb(26, 44, 48) : C_PANEL);
  if (editing(P_GAIN)) selectedOutline(s, 8, 0, 208, 214, 10);
  const int cx = 112, cy = 104;
  arc(s, cx, cy, 62, 76, 135.0f, 405.0f, C_GRAPH);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(editing(P_GAIN) ? C_TEAL : C_DIM);
  s.drawString("GAIN", 20, 10);
  if (!isnan(d.gain)) {
    arc(s, cx, cy, 62, 76, 135.0f, 135.0f + 270.0f * constrain(d.gain, 0.0f, 1.0f), cc);
    char t[12];
    snprintf(t, sizeof(t), "%+.1f", preampDb(d.gain));
    s.setFont(&fonts::FreeSansBold18pt7b);
    s.setTextDatum(middle_center);
    s.setTextColor(C_TEXT);
    s.drawString(t, cx, cy - 2);
    s.setFont(&fonts::DejaVu12);
    s.setTextColor(C_FAINT);
    s.drawString("dB", cx, cy + 26);
    s.setFont(&fonts::DejaVu9);
    s.drawString("-12", cx - 52, cy + 66);
    s.drawString("+60", cx + 52, cy + 66);
    zone(8, 0, 208, 214, Z_EDIT, P_GAIN);
  } else {
    s.setFont(&fonts::DejaVu12);
    s.setTextDatum(middle_center);
    s.setTextColor(C_FAINT);
    s.drawString(snap.sel == MAIN_STRIP ? "no preamp" : "--", cx, cy);
  }
}

void drawPreampPageB(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  static const uint8_t IDS[] = {P_PHANTOM, P_INVERT, P_HPON, P_HPF, P_LR, P_PAN};
  for (int i = 0; i < 6; i++) tile(s, (i % 2) * 124, 4 + (i / 2) * 72, 120, 66, IDS[i]);
  // pan position bar inside its tile
  float pan = uiParam(P_PAN, 0);
  if (!isnan(pan)) {
    int bx = 124 + 64, bw = 46, by = 4 + 2 * 72 + 26;
    s.fillSmoothRoundRect(bx, by, bw, 4, 2, C_GRAPH);
    s.drawFastVLine(bx + bw / 2, by - 3, 10, C_GRID0);
    s.fillSmoothCircle(bx + (int)lroundf(pan * bw), by + 2, 4, C_TEXT);
  }
}

// ---------- home screen ----------

// The default view: tabs of everything the mixer reports. Channel strips
// open the channel view; mutes, mute groups and the bus/FX/DCA/headphone
// levels can be changed right here.

const char *const HOME_TABS[] = {"1-8", "9-16", "RTN", "BUS", "DCA", "METERS", "FX", "MIXER"};
const int NUM_HOME_TABS = 8;
enum HomeTab { T_BANK1, T_BANK2, T_RTN, T_BUS, T_DCA, T_METERS, T_FX, T_MIXER };

const char *const PHONES_SRC[] = {"OFF", "LR", "LR PFL", "LR AFL", "AUX", "USB 17-18", "BUS 1", "BUS 2",
                                  "BUS 3", "BUS 4", "BUS 5", "BUS 6", "BUS 1-2", "BUS 3-4", "BUS 5-6"};
const char *const FX_CODES[61] = {
  "HALL", "AMBI", "RPLT", "ROOM", "CHAM", "PLAT", "VREV", "VRM", "GATE", "RVRS", "DLY", "3TAP", "4TAP",
  "CRS", "FLNG", "PHAS", "DIMC", "FILT", "ROTA", "PAN", "SUB", "D/RV", "CR/R", "FL/R", "D/CR", "D/FL",
  "MODD", "GEQ2", "GEQ", "TEQ2", "TEQ", "DES2", "DES", "P1A", "P1A2", "PQ5", "PQ5S", "WAVD", "LIM",
  "CMB", "CMB2", "FAC", "FAC1M", "FAC2", "LEC", "LEC2", "ULC", "ULC2", "ENH2", "ENH", "EXC2", "EXC",
  "IMG", "EDI", "SON", "AMP2", "AMP", "DRV2", "DRV", "PIT2", "PIT"};
const char *const FX_NAMES[61] = {
  "Hall Reverb", "Ambience", "Rich Plate", "Room Reverb", "Chamber", "Plate Reverb", "Vintage Reverb",
  "Vintage Room", "Gated Reverb", "Reverse Reverb", "Stereo Delay", "3-Tap Delay", "Rhythm Delay",
  "Stereo Chorus", "Stereo Flanger", "Stereo Phaser", "Dimension Chorus", "Mood Filter", "Rotary Speaker",
  "Tremolo / Panner", "Sub Octaver", "Delay + Reverb", "Chorus + Reverb", "Flanger + Reverb",
  "Delay + Chorus", "Delay + Flanger", "Modulation Delay", "Dual GEQ", "Stereo GEQ", "Dual TrueEQ",
  "Stereo TrueEQ", "Dual De-esser", "Stereo De-esser", "Vintage EQ", "Dual Vintage EQ", "Mid EQ",
  "Stereo Mid EQ", "Wave Designer", "Precision Limiter", "Combinator", "Dual Combinator",
  "Fair Compressor", "Fair Comp (M/S)", "Dual Fair Comp", "Leisure Comp", "Dual Leisure Comp",
  "Ultimo Comp", "Dual Ultimo Comp", "Dual Enhancer", "Stereo Enhancer", "Dual Exciter",
  "Stereo Exciter", "Stereo Imager", "Edison EX1", "Sound Maxer", "Dual Guitar Amp",
  "Stereo Guitar Amp", "Dual Tube Stage", "Stereo Tube Stage", "Dual Pitch", "Stereo Pitch"};

// One strip on the home screen: a channel (ch >= 0) or one of others[] (oi >= 0).
struct HomeStrip {
  const char *label, *name;
  int8_t color, on;
  float fader, meter;
  int ch, oi;
};

HomeStrip channelStrip(int i) {
  const StripView &v = snap.s[i];
  return {v.label, v.name, v.color, v.on, v.haveFader ? v.fader : NAN, v.meter, i, -1};
}

HomeStrip otherStrip(int oi, float meter) {
  const Other &o = snap.oth[oi];
  return {o.label, o.name, o.color, o.on, o.fader, meter, -1, oi};
}

float meterPair(int a, int b) {
  float x = snap.meters40[a], y = snap.meters40[b];
  return isnan(x) ? y : isnan(y) ? x : max(x, y);
}

// A vertical strip: name, (channel) source and badges, meter + fader, dB,
// pan (channels), and a mute button.
// Which bank fader (0-7) a home strip is on right now, or -1.
int faderFor(const HomeStrip &h) {
  if (!snap.fbConnected) return -1;
  for (int i = 0; i < BANK_FADERS && i < max(snap.fbFaders, 1); i++) {
    BankItem it = bankItem(snap.bank, i);
    if ((it.kind == 1 && it.idx == h.ch && h.ch >= 0) || (it.kind == 2 && it.idx == h.oi && h.oi >= 0)) return i;
  }
  return -1;
}

void homeStrip(LGFX_Sprite &s, int x, int w, const HomeStrip &h) {
  const int top = 4, bottom = 272;
  int fader = faderFor(h);
  bool hand = fader >= 0 && (snap.fbTouch & (1 << fader));
  bool sel = h.ch >= 0 && h.ch == snap.sel;
  bool edit = h.oi >= 0 && editing(P_OTHER, h.oi);
  uint16_t cc = chanColor(h.color);
  s.fillSmoothRoundRect(x, top, w, bottom - top, 6, sel || edit ? rgb(26, 36, 48) : C_PANEL);
  if (sel) { s.drawRoundRect(x, top, w, bottom - top, 6, cc); s.drawRoundRect(x + 1, top + 1, w - 2, bottom - top - 2, 5, cc); }
  if (edit) selectedOutline(s, x, top, w, bottom - top, 6);
  s.fillRect(x + 4, top + 4, w - 8, 3, cc);
  if (fader >= 0) {  // on a motorized fader: its number, lit while a hand is on it
    char t[3];
    snprintf(t, sizeof(t), "%d", fader + 1);
    s.fillSmoothCircle(x + 9, top + 14, 6, hand ? C_TEXT : C_TEAL);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(middle_center);
    s.setTextColor(C_BG);
    s.drawString(t, x + 9, top + 15);
  }

  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_center);
  s.setTextColor(C_DIM);
  s.drawString(h.label, x + w / 2, top + 10);
  const char *name = h.name && h.name[0] ? h.name : "";
  s.setFont(&fonts::DejaVu12);
  if (s.textWidth(name) > w - 6) s.setFont(&fonts::DejaVu9);
  s.setTextColor(C_TEXT);
  char shortName[16];
  strlcpy(shortName, name, sizeof(shortName));
  while (strlen(shortName) > 1 && s.textWidth(shortName) > w - 6) shortName[strlen(shortName) - 1] = 0;
  s.drawString(shortName, x + w / 2, top + 22);

  int y = top + 38;
  if (h.ch >= 0) {
    const Detail &d = snap.dAll[h.ch];
    char src[16] = "";
    if (h.ch < 16 && d.insrc >= 0) snprintf(src, sizeof(src), d.insrc ? "IN %d" : "OFF", d.insrc);
    if (d.fxslot > 0 && d.insOn == 1) {  // an insert effect on this channel
      char ins[8];
      snprintf(ins, sizeof(ins), " FX%d%c", (d.fxslot + 1) / 2, d.fxslot % 2 ? 'A' : 'B');
      strlcat(src, ins, sizeof(src));
    }
    s.setFont(&fonts::DejaVu9);
    s.setTextColor(C_FAINT);
    s.drawString(src, x + w / 2, y);
    // badges: 48V, EQ, gate, comp, solo
    struct { const char *t; bool on; uint16_t c; } b[] = {
      {"48V", d.phantom == 1, C_RED}, {"EQ", d.eqOn == 1, C_GREEN},
      {"G", d.gateOn == 1, C_RED}, {"C", d.dynOn == 1, C_ORANGE}};
    int bx = x + 4, by = y + 13, bw = (w - 8 - 3) / 4;
    for (int k = 0; k < 4; k++) {
      int xx = bx + k * (bw + 1);
      s.fillSmoothRoundRect(xx, by, bw, 11, 3, b[k].on ? b[k].c : C_GRAPH);
      s.setTextColor(b[k].on ? C_BG : C_FAINT);
      s.setTextDatum(middle_center);
      if (bw >= 12 || k) s.drawString(k == 0 && bw < 18 ? "P" : b[k].t, xx + bw / 2, by + 6);
    }
    if (d.solo == 1) {
      s.fillSmoothCircle(x + w - 8, top + 12, 5, C_AMBER);
      s.setTextColor(C_BG);
      s.setTextDatum(middle_center);
      s.drawString("S", x + w - 8, top + 13);
    }
  }

  // meter and fader
  const int fTop = top + 70, fBot = 212, fh = fBot - fTop;
  int mx = x + 6, fx = x + w / 2 + 6;
  s.fillRect(mx, fTop, 5, fh, C_GRAPH);
  if (!isnan(h.meter) && h.meter > -60.0f) {
    int mh = (int)lroundf((h.meter + 60.0f) / 60.0f * fh);
    for (int yy = 0; yy < mh; yy += 3) {
      float db = -60.0f + 60.0f * yy / fh;
      s.fillRect(mx, fBot - yy - 2, 5, 2, meterColor(db));
    }
  }
  int unity = fBot - (int)lroundf(0.75f * fh);
  s.drawFastHLine(fx - 9, unity, 18, C_GRID0);
  s.fillSmoothRoundRect(fx - 2, fTop, 4, fh, 2, C_GRAPH);
  if (!isnan(h.fader)) {
    int fy = fBot - (int)lroundf(h.fader * fh);
    int capW = min(w - 22, 26);
    s.fillSmoothRoundRect(fx - capW / 2, fy - 6, capW, 12, 3, edit || hand ? C_TEXT : rgb(176, 188, 202));
    if (hand) s.drawRoundRect(fx - capW / 2 - 1, fy - 7, capW + 2, 14, 4, C_TEAL);
    s.fillRect(fx - capW / 2 + 3, fy - 1, capW - 6, 2, cc);
  }
  char t[10];
  if (isnan(h.fader)) strcpy(t, "--");
  else if (h.fader <= 0.0f) strcpy(t, "-inf");
  else snprintf(t, sizeof(t), "%+.1f", faderToDb(h.fader));
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(middle_center);
  s.setTextColor(edit ? C_TEAL : C_TEXT);
  s.drawString(t, x + w / 2, fBot + 10);

  if (h.ch >= 0 && !isnan(snap.dAll[h.ch].pan)) {  // pan
    int px0 = x + 6, pw = w - 12, py = fBot + 22;
    float pan = snap.dAll[h.ch].pan;
    s.fillRect(px0, py, pw, 3, C_GRAPH);
    s.drawFastVLine(px0 + pw / 2, py - 2, 7, C_GRID0);
    s.fillSmoothCircle(px0 + (int)lroundf(pan * pw), py + 1, 3, C_DIM);
  }

  // mute button
  int my = bottom - 24;
  bool muted = h.on == 0;
  s.fillSmoothRoundRect(x + 3, my, w - 6, 20, 5, muted ? C_RED : C_GRAPH);
  s.setTextColor(muted ? C_TEXT : h.on < 0 ? C_GRID : C_FAINT);
  s.drawString(w >= 50 ? "MUTE" : "M", x + w / 2, my + 10);

  int muteArg = h.ch >= 0 ? h.ch : 32 + h.oi;
  zone(x, my - 4, w, 30, Z_MUTE_ANY, 0, muteArg);
  if (h.ch >= 0) zone(x, top, w, my - top - 4, Z_OPEN_CH, 0, h.ch);
  else zone(x, top, w, my - top - 4, Z_EDIT, P_OTHER, h.oi);
}

// The fader bank a home tab puts on the motorized faders, or -1.
int tabBank(int tab) {
  switch (tab) {
    case T_BANK1: return BANK_1_8;
    case T_BANK2: return BANK_9_16;
    case T_RTN:   return BANK_RTN;
    case T_BUS:   return BANK_BUS;
    case T_DCA:   return BANK_DCA;
  }
  return -1;
}

void drawHomeTabs(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  const int w = 56;
  for (int i = 0; i < NUM_HOME_TABS; i++) {
    int x = 2 + i * 60;
    bool sel = homeTab == i;
    s.fillSmoothRoundRect(x, 4, w, 32, 7, sel ? C_TEAL : C_PANEL);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(middle_center);
    s.setTextColor(sel ? C_BG : C_DIM);
    s.drawString(HOME_TABS[i], x + w / 2, 21);
    if (snap.fbConnected && tabBank(i) == snap.bank)  // the faders are on this tab
      s.fillRect(x + w / 2 - 10, 32, 20, 3, sel ? C_BG : C_TEAL);
    zone(x - 2, 0, w + 4, 40, Z_TAB, 0, i);
  }
}

void drawBank(LGFX_Sprite &s, int first) {
  for (int i = 0; i < 8; i++) homeStrip(s, 3 + i * 59, 56, channelStrip(first + i));
}

void drawReturns(LGFX_Sprite &s) {
  const int w = 76, pitch = 79;
  homeStrip(s, 3, w, channelStrip(AUX_STRIP));
  for (int i = 0; i < 4; i++) homeStrip(s, 3 + (i + 1) * pitch, w, otherStrip(OTHER_RTN + i, meterPair(18 + i * 2, 19 + i * 2)));
  homeStrip(s, 3 + 5 * pitch, w, channelStrip(MAIN_STRIP));
}

void drawBuses(LGFX_Sprite &s) {
  for (int i = 0; i < 6; i++) homeStrip(s, 3 + i * 47, 45, otherStrip(OTHER_BUS + i, snap.meters40[26 + i]));
  for (int i = 0; i < 4; i++) homeStrip(s, 3 + 6 * 47 + 8 + i * 47, 45, otherStrip(OTHER_FXSEND + i, snap.meters40[32 + i]));
}

void drawDca(LGFX_Sprite &s) {
  for (int i = 0; i < 4; i++) homeStrip(s, 3 + i * 66, 63, otherStrip(OTHER_DCA + i, NAN));
  // mute groups
  int x0 = 272;
  s.fillSmoothRoundRect(x0, 4, 204, 128, 8, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString("MUTE GROUPS", x0 + 10, 12);
  for (int g = 0; g < 4; g++) {
    int bx = x0 + 10 + (g % 2) * 96, by = 30 + (g / 2) * 50;
    int8_t on = snap.mi.muteGrp[g];
    s.fillSmoothRoundRect(bx, by, 88, 42, 8, on == 1 ? C_RED : C_GRAPH);
    s.setFont(&fonts::FreeSansBold9pt7b);
    s.setTextDatum(middle_center);
    s.setTextColor(on == 1 ? C_TEXT : on < 0 ? C_GRID : C_DIM);
    char t[8];
    snprintf(t, sizeof(t), "MG %d", g + 1);
    s.drawString(t, bx + 44, by + 22);
    zone(bx - 2, by - 2, 92, 46, Z_MUTEGRP, 0, g);
  }
  // headphones
  bool edit = editing(P_OTHER, OTHER_PHONES);
  s.fillSmoothRoundRect(x0, 138, 204, 134, 8, edit ? rgb(26, 44, 48) : C_PANEL);
  if (edit) selectedOutline(s, x0, 138, 204, 134, 8);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(edit ? C_TEAL : C_DIM);
  s.drawString("HEADPHONES", x0 + 10, 146);
  char t[16];
  s.setTextColor(C_FAINT);
  s.drawString("SOURCE", x0 + 10, 166);
  int8_t src = snap.mi.phonesSrc;
  s.setFont(&fonts::DejaVu12);
  s.setTextColor(C_TEXT);
  s.drawString(src >= 0 && src < 15 ? PHONES_SRC[src] : "--", x0 + 10, 180);
  float lvl = snap.mi.phonesLevel;
  if (isnan(lvl)) strcpy(t, "--");
  else if (lvl <= 0) strcpy(t, "-inf dB");
  else snprintf(t, sizeof(t), "%+.1f dB", faderToDb(lvl));
  s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextDatum(bottom_left);
  s.drawString(t, x0 + 10, 236);
  int bx = x0 + 10, bw = 184, by = 248;
  s.fillSmoothRoundRect(bx, by, bw, 8, 4, C_GRAPH);
  if (!isnan(lvl)) s.fillSmoothRoundRect(bx, by, max(8, (int)lroundf(lvl * bw)), 8, 4, C_TEAL);
  zone(x0, 138, 204, 134, Z_EDIT, P_OTHER, OTHER_PHONES);
}

float peaks[40];
uint32_t peakAt[40];

void drawMeters(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  s.fillSmoothRoundRect(3, 4, 474, 268, 8, C_PANEL);
  struct Group { const char *name; int first, count; } groups[] = {
    {"CHANNELS", 0, 16}, {"AUX", 16, 2}, {"FX RETURNS", 18, 8}, {"BUSES", 26, 6},
    {"SENDS", 32, 4}, {"MAIN", 36, 2}, {"MON", 38, 2}};
  const int top = 26, bottom = 238, h = bottom - top, bw = 8, pitch = 10;
  auto yFor = [&](float db) { return bottom - (int)lroundf((constrain(db, -60.0f, 0.0f) + 60.0f) / 60.0f * h); };
  for (int db : {0, -6, -12, -24, -36, -48}) {
    s.drawFastHLine(10, yFor(db), 460, db == 0 ? C_GRID0 : C_GRID);
  }
  int x = 12;
  uint32_t now = millis();
  for (const Group &g : groups) {
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_left);
    s.setTextColor(C_DIM);
    int gw = g.count * pitch;
    s.setTextDatum(top_center);
    s.drawString(g.count > 3 ? g.name : g.name, x + gw / 2 - 1, 10);
    for (int k = 0; k < g.count; k++) {
      int i = g.first + k, bx = x + k * pitch;
      s.fillRect(bx, top, bw, h, C_GRAPH);
      float m = snap.meters40[i];
      if (!isnan(m) && m > -60.0f) {
        for (int yy = yFor(m); yy < bottom; yy += 3) {
          float db = -60.0f + 60.0f * (bottom - yy) / h;
          s.fillRect(bx, yy, bw, 2, meterColor(db));
        }
        if (isnan(peaks[i]) || m >= peaks[i] || now - peakAt[i] > 1500) { peaks[i] = m; peakAt[i] = now; }
      }
      if (!isnan(peaks[i]) && peaks[i] > -60.0f) s.fillRect(bx, yFor(peaks[i]), bw, 2, C_TEXT);
      char t[4];
      if (g.first == 0) snprintf(t, sizeof(t), "%d", k + 1);
      else if (g.count == 2) snprintf(t, sizeof(t), "%c", k ? 'R' : 'L');
      else if (g.first == 18) snprintf(t, sizeof(t), "%d", k / 2 + 1);
      else snprintf(t, sizeof(t), "%d", k + 1);
      s.setTextDatum(top_center);
      s.setTextColor(C_FAINT);
      s.drawString(t, bx + bw / 2, bottom + 6);
    }
    x += gw + 8;
  }
  s.setTextDatum(middle_left);
  s.setTextColor(C_FAINT);
  s.drawString("0", 6, yFor(0));
}

void cell(LGFX_Sprite &s, int x, int y, int w, int h, const char *label, const char *value,
          bool dim = false) {
  s.fillSmoothRoundRect(x, y, w, h, 5, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(middle_left);
  s.setTextColor(C_DIM);
  s.drawString(label, x + 8, y + h / 2);
  s.setTextDatum(middle_right);
  s.setFont(&fonts::DejaVu12);
  s.setTextColor(dim ? C_FAINT : C_TEXT);
  s.drawString(value, x + w - 8, y + h / 2);
}



void drawFx(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  for (int f = 0; f < 4; f++) {
    int x = 4 + (f % 2) * 238, y = 4 + (f / 2) * 136, w = 234, h = 132;
    s.fillSmoothRoundRect(x, y, w, h, 8, C_PANEL);
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_left);
    s.setTextColor(C_DIM);
    char t[48];
    snprintf(t, sizeof(t), "FX %d", f + 1);
    s.drawString(t, x + 10, y + 8);
    int8_t ty = snap.mi.fxType[f];
    s.setFont(&fonts::FreeSansBold12pt7b);
    s.setTextColor(C_TEXT);
    s.drawString(ty >= 0 && ty < 61 ? FX_NAMES[ty] : "--", x + 10, y + 22);
    if (ty >= 0 && ty < 61) {
      s.setFont(&fonts::DejaVu9);
      s.setTextDatum(top_right);
      s.setTextColor(C_FAINT);
      s.drawString(FX_CODES[ty], x + w - 10, y + 8);
    }
    // send and return levels (tap to adjust with the knob)
    for (int k = 0; k < 2; k++) {
      int oi = k ? OTHER_RTN + f : OTHER_FXSEND + f;
      const Other &o = snap.oth[oi];
      bool edit = editing(P_OTHER, oi);
      int by = y + 56 + k * 26, bx = x + 64, bw = 100;
      if (edit) selectedOutline(s, x + 4, by - 4, w - 8, 24, 5);
      s.setFont(&fonts::DejaVu9);
      s.setTextDatum(middle_left);
      s.setTextColor(edit ? C_TEAL : C_DIM);
      s.drawString(k ? "RETURN" : "SEND", x + 10, by + 8);
      s.fillSmoothRoundRect(bx, by + 4, bw, 8, 4, C_GRAPH);
      if (!isnan(o.fader)) s.fillSmoothRoundRect(bx, by + 4, max(8, (int)lroundf(o.fader * bw)), 8, 4, k ? C_PURPLE : C_TEAL);
      if (isnan(o.fader)) strcpy(t, "--");
      else if (o.fader <= 0) strcpy(t, "-inf");
      else snprintf(t, sizeof(t), "%+.1f", faderToDb(o.fader));
      s.setTextDatum(middle_right);
      s.setTextColor(C_TEXT);
      s.drawString(t, x + w - 36, by + 8);
      bool muted = o.on == 0;
      s.fillSmoothRoundRect(x + w - 30, by, 22, 16, 4, muted ? C_RED : C_GRAPH);
      s.setTextDatum(middle_center);
      s.setTextColor(muted ? C_TEXT : C_FAINT);
      s.drawString("M", x + w - 19, by + 8);
      zone(x + w - 34, by - 4, 30, 24, Z_MUTE_ANY, 0, 32 + oi);
      zone(x + 4, by - 4, w - 40, 24, Z_EDIT, P_OTHER, oi);
    }
    // channels inserting this FX
    char ins[48] = "";
    for (int c = 0; c < 16; c++) {
      int8_t slot = snap.dAll[c].fxslot;
      if (slot > 0 && (slot + 1) / 2 == f + 1) {
        char one[8];
        snprintf(one, sizeof(one), "%s%d", ins[0] ? ", " : "", c + 1);
        strlcat(ins, one, sizeof(ins));
      }
    }
    s.setFont(&fonts::DejaVu9);
    s.setTextDatum(top_left);
    s.setTextColor(C_FAINT);
    snprintf(t, sizeof(t), ins[0] ? "INSERTED ON CH %s" : "NOT INSERTED ON A CHANNEL", ins);
    s.drawString(t, x + 10, y + h - 18);
  }
}

void drawMixerInfo(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  char v[48];
  snprintf(v, sizeof(v), "%s %s", snap.mixerName, snap.mixerModel);
  cell(s, 4, 4, 236, 30, "MIXER", v);
  cell(s, 4, 38, 236, 30, "MIXER IP", snap.mixer);
  cell(s, 4, 72, 236, 30, "NETWORK", snap.online ? snap.netSsid : "--");
  cell(s, 4, 106, 236, 30, "CONTROLLER", snap.ip);
  if (snap.mi.snapIndex >= 0 || snap.mi.snapName[0]) {
    snprintf(v, sizeof(v), snap.mi.snapIndex >= 0 ? "#%d %s" : "%.0s%s", snap.mi.snapIndex, snap.mi.snapName);
    cell(s, 4, 140, 236, 30, "SNAPSHOT", v);
  } else {
    cell(s, 4, 140, 236, 30, "SNAPSHOT", "--", true);
  }
  // automix
  snprintf(v, sizeof(v), "X %s   Y %s", snap.mi.amix[0] == 1 ? "ON" : "off", snap.mi.amix[1] == 1 ? "ON" : "off");
  cell(s, 4, 174, 236, 30, "AUTOMIX", v, snap.mi.amix[0] < 0);

  // links
  s.fillSmoothRoundRect(246, 4, 230, 200, 8, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString("CHANNEL LINKS", 256, 12);
  for (int i = 0; i < 8; i++) {
    int bx = 256 + (i % 4) * 54, by = 28 + (i / 4) * 34;
    bool on = snap.mi.chLink[i] == 1;
    s.fillSmoothRoundRect(bx, by, 50, 28, 6, on ? C_TEAL : C_GRAPH);
    s.setTextDatum(middle_center);
    s.setTextColor(on ? C_BG : C_FAINT);
    char t[8];
    snprintf(t, sizeof(t), "%d-%d", i * 2 + 1, i * 2 + 2);
    s.drawString(t, bx + 25, by + 14);
  }
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString("BUS LINKS", 256, 104);
  for (int i = 0; i < 3; i++) {
    int bx = 256 + i * 72, by = 120;
    bool on = snap.mi.busLink[i] == 1;
    s.fillSmoothRoundRect(bx, by, 66, 28, 6, on ? C_PURPLE : C_GRAPH);
    s.setTextDatum(middle_center);
    s.setTextColor(on ? C_BG : C_FAINT);
    char t[8];
    snprintf(t, sizeof(t), "%d-%d", i * 2 + 1, i * 2 + 2);
    s.drawString(t, bx + 33, by + 14);
  }
  // settings
  s.fillSmoothRoundRect(4, 214, 236, 56, 10, C_PANEL);
  s.drawRoundRect(4, 214, 236, 56, 10, C_BORDER);
  drawGear(s, 34, 242, C_DIM, C_PANEL);
  s.setFont(&fonts::FreeSansBold12pt7b);
  s.setTextDatum(middle_left);
  s.setTextColor(C_TEXT);
  s.drawString("Settings", 58, 243);
  // motorized fader board
  s.fillSmoothRoundRect(246, 214, 230, 56, 10, C_PANEL);
  s.setFont(&fonts::DejaVu9);
  s.setTextDatum(top_left);
  s.setTextColor(C_DIM);
  s.drawString("FADER BOARD", 256, 222);
  char fb[40];
  if (snap.fbConnected)
    snprintf(fb, sizeof(fb), "%d faders, %s", snap.fbFaders, snap.fbHasTouch ? "touch sensor" : "motion touch");
  else
    snprintf(fb, sizeof(fb), "not connected");
  s.setFont(&fonts::DejaVu12);
  s.setTextColor(snap.fbConnected ? C_TEXT : C_FAINT);
  s.drawString(fb, 256, 240);
  zone(4, 214, 236, 56, Z_GEAR);
}

void drawHomeBody(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  switch (homeTab) {
    case T_BANK1:  drawBank(s, 0); break;
    case T_BANK2:  drawBank(s, 8); break;
    case T_RTN:    drawReturns(s); break;
    case T_BUS:    drawBuses(s); break;
    case T_DCA:    drawDca(s); break;
    case T_METERS: drawMeters(s); break;
    case T_FX:     drawFx(s); break;
    case T_MIXER:  drawMixerInfo(s); break;
  }
}

// Insert page: pick which effect slot is inserted on the selected channel.
void drawInsertPage(LGFX_Sprite &s) {
  s.fillScreen(C_BG);
  const Detail &d = snap.d;
  if (snap.sel >= 16) {
    s.setFont(&fonts::DejaVu12);
    s.setTextDatum(middle_center);
    s.setTextColor(C_FAINT);
    s.drawString("Insert effects are available on channels 1-16", 240, 100);
    return;
  }
  for (int slot = 0; slot <= 8; slot++) {
    int x = 6 + (slot % 3) * 158, y = 4 + (slot / 3) * 72, w = 152, h = 66;
    bool sel = d.fxslot == slot;
    bool live = sel && (slot == 0 || d.insOn == 1);
    s.fillSmoothRoundRect(x, y, w, h, 8, sel ? (live ? mix565(C_PANEL, C_PURPLE, 0.45f) : rgb(40, 40, 60)) : C_PANEL);
    s.drawRoundRect(x, y, w, h, 8, sel ? C_PURPLE : C_BORDER);
    char t[32];
    s.setFont(&fonts::FreeSansBold12pt7b);
    s.setTextDatum(top_left);
    s.setTextColor(C_TEXT);
    if (slot == 0) {
      s.drawString("OFF", x + 12, y + 10);
      s.setFont(&fonts::DejaVu9);
      s.setTextColor(C_FAINT);
      s.drawString("no insert effect", x + 12, y + 40);
    } else {
      int fx = (slot + 1) / 2;
      snprintf(t, sizeof(t), "FX %d%c", fx, slot % 2 ? 'A' : 'B');
      s.drawString(t, x + 12, y + 8);
      int8_t ty = snap.mi.fxType[fx - 1];
      s.setFont(&fonts::DejaVu9);
      s.setTextColor(C_DIM);
      s.drawString(ty >= 0 && ty < 61 ? FX_NAMES[ty] : "--", x + 12, y + 34);
      char users[24] = "";
      for (int c = 0; c < 16; c++) {
        if (c == snap.sel || snap.dAll[c].fxslot != slot || snap.dAll[c].insOn != 1) continue;
        char one[8];
        snprintf(one, sizeof(one), "%s%d", users[0] ? "," : "", c + 1);
        strlcat(users, one, sizeof(users));
      }
      s.setTextColor(users[0] ? C_AMBER : C_FAINT);
      snprintf(t, sizeof(t), users[0] ? "also on CH %s" : "free", users);
      s.drawString(t, x + 12, y + 48);
    }
    zone(x, y, w, h, Z_SETINS, 0, slot);
  }
}

uint32_t homeBodyHash() {
  Hash h;
  h.add(homeTab).add(snap.sel).add(snap.editId).add(snap.editIdx).add(snap.bank).add(snap.fbConnected)
   .add(snap.fbTouch).add(snap.fbFaders).add(snap.fbHasTouch);
  bool meters = homeTab <= T_BUS || homeTab == T_METERS;
  for (int i = 0; i < NUM_STRIPS; i++) {
    const StripView &v = snap.s[i];
    const Detail &d = snap.dAll[i];
    h.str(v.name).add(v.on).add(v.color).q(v.fader, 400).add(d.insrc).add(d.fxslot).add(d.insOn)
     .add(d.phantom).add(d.eqOn).add(d.gateOn).add(d.dynOn).add(d.solo).q(d.pan, 100);
    if (meters) h.q(v.meter, 0.5f);
  }
  for (const Other &o : snap.oth) h.str(o.name).add(o.on).add(o.color).q(o.fader, 400);
  if (meters) for (float m : snap.meters40) h.q(m, 0.5f);
  if (homeTab == T_METERS) h.add(millis() / 500);  // let peak holds fall
  const MixerInfo &m = snap.mi;
  h.add(m);  // routing, links, FX types, mute groups...
  h.str(snap.ip).str(snap.netSsid).str(snap.mixer).str(snap.mixerName).add(snap.online);
  return h.h;
}

// ---------- screens ----------

enum View { V_STATUS, V_MAIN, V_PICKER, V_KEYBOARD, V_SETTINGS, V_SYNC, V_PAGE, V_HOME };
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

void sendAction(UiActionType type, int arg = 0, int arg2 = 0) {
  UiAction a = {type, (int8_t)arg, (uint8_t)arg2};
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
  if (!channelOpen) return V_HOME;
  return page != PG_NONE ? V_PAGE : V_MAIN;
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
  tft.setTextDatum(middle_left);  // button() leaves it centred

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

void openPage(Page pg) {
  page = pg;
  if (pg == PG_EQ && snap.editId >= P_EQ_TYPE && snap.editId <= P_EQ_Q) pageBand = snap.editIdx & 3;
  else if (pg == PG_EQ && snap.editId == P_HPF) pageBand = 4;
}

// Taps on the overview and pages: the smallest zone under the finger wins.
void zoneTap(int x, int y, const char *&what) {
  Panel *const *list = view == V_PAGE ? PAGE_PANELS : view == V_HOME ? HOME_PANELS : MAIN_PANELS;
  int count = view == V_PAGE ? 4 : view == V_HOME ? 2 : 7;
  const Zone *best = nullptr;
  int bestArea = INT32_MAX;
  for (int p = 0; p < count; p++) {
    const Panel &pn = *list[p];
    if (!pn.w) continue;
    for (int i = 0; i < pn.nz; i++) {
      const Zone &z = pn.zones[i];
      if (!inside(x, y, z.x, z.y, z.w, z.h, 3)) continue;
      int area = z.w * z.h;
      if (area < bestArea) { best = &z; bestArea = area; }
    }
  }
  if (!best) return;
  Zone z = *best;
  switch (z.act) {
    case Z_EDIT:
      if (snap.editId == z.id && snap.editIdx == z.idx) sendAction(UI_EDIT, P_NONE);
      else sendAction(UI_EDIT, z.id, z.idx);
      what = "edit";
      break;
    case Z_STOP:   sendAction(UI_EDIT, P_NONE); what = "stop editing"; break;
    case Z_TOGGLE: sendAction(UI_TOGGLE, z.id, z.idx); what = "toggle"; break;
    case Z_PAGE:   openPage((Page)z.id); what = "page"; break;
    case Z_MUTE_ANY: sendAction(UI_MUTE_ANY, 0, z.idx); what = "mute"; break;
    case Z_SETINS:   sendAction(UI_SET, P_INS_SLOT, z.idx); what = "insert"; break;
    case Z_MUTEGRP:  sendAction(UI_MUTEGRP, 0, z.idx); what = "mute group"; break;
    case Z_TAB:
      homeTab = z.idx;
      if (tabBank(z.idx) >= 0) sendAction(UI_BANK, 0, tabBank(z.idx));  // faders follow the tab
      what = "tab";
      break;
    case Z_OPEN_CH:
      sendAction(UI_SELECT, z.idx);
      channelOpen = true;
      what = snap.s[z.idx].label;
      break;
    case Z_HOME:
      channelOpen = false;
      page = PG_NONE;
      if (snap.editId != P_OTHER) sendAction(UI_EDIT, P_NONE);
      what = "home";
      break;
    case Z_BACK:
      page = PG_NONE;
      // Settings only editable on their page stop being edited when you leave it
      if (snap.editId != P_FADER && snap.editId != P_GAIN && snap.editId != P_PAN)
        sendAction(UI_EDIT, P_NONE);
      what = "back";
      break;
    case Z_STRIP:  sendAction(UI_SELECT, z.idx); what = snap.s[z.idx].label; break;
    case Z_GEAR:   screen = SCR_SETTINGS; what = "settings"; break;
    case Z_MUTE:
      sendAction(UI_MUTE);
      pressedButton = 0;
      pressedAt = millis();
      what = "MUTE";
      break;
    case Z_SOLO:
      sendAction(UI_SOLO);
      pressedButton = 1;
      pressedAt = millis();
      what = "SOLO";
      break;
    case Z_BAND: {  // EQ page: pick a band, and point the knob at the same setting of it
      pageBand = z.idx;
      uint8_t id = snap.editId;
      if (z.idx == 4) sendAction(UI_EDIT, P_HPF);
      else if (id >= P_EQ_TYPE && id <= P_EQ_Q) sendAction(UI_EDIT, id, z.idx);
      else sendAction(UI_EDIT, P_EQ_G, z.idx);
      what = "band";
      break;
    }
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
    case V_MAIN:
    case V_HOME:
    case V_PAGE:     zoneTap(x, y, what); break;
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

struct Job { Panel *p; uint32_t h; void (*draw)(LGFX_Sprite &); };

void runJobs(Job *jobs, int n) {
  for (int i = 0; i < n; i++) {
    Job &j = jobs[i];
    if (!j.p->w || j.p->hash == j.h) continue;
    j.p->hash = j.h;
    beginZones(*j.p);
    j.draw(j.p->spr);
    zonePanel = nullptr;
    j.p->spr.pushSprite(&tft, j.p->x, j.p->y);
  }
}

uint32_t detailHash() {
  Hash h;
  const Detail &d = snap.d;
  h.add(snap.sel).add(d.color).add(d.eqOn).add(d.hpOn).q(d.hpf, 1000).add(d.lr).add(d.phantom)
   .add(d.fxslot).add(d.insOn)
   .add(d.invert).q(d.pan, 1000).q(d.gain, 1000).add(d.gateOn).add(d.dynOn).add(d.dynRatio)
   .q(d.gateThr, 1000).q(d.gateRange, 1000).q(d.gateAtt, 1000).q(d.gateHold, 1000).q(d.gateRel, 1000)
   .q(d.dynThr, 1000).q(d.dynKnee, 1000).q(d.dynAtt, 1000).q(d.dynRel, 1000).q(d.dynMgain, 1000)
   .q(d.dynMix, 1000).add(snap.editId).add(snap.editIdx);
  for (const EqBand &b : d.eq) h.add(b.type).q(b.f, 1000).q(b.g, 1000).q(b.q, 1000);
  for (float x : d.sends) h.q(x, 1000);
  return h.h;
}

uint32_t stripsHash() {
  Hash ho;
  ho.add(snap.sel);
  for (const StripView &sv : snap.s) ho.q(sv.fader, 400).q(sv.meter, 1).add(sv.on).add(sv.color).add(sv.haveFader);
  return ho.h;
}

void drawMain() {
  const StripView &v = snap.s[snap.sel];
  const Detail &d = snap.d;
  bool flash = millis() - pressedAt < PRESS_FLASH_MS;
  uint32_t dh = detailHash();
  char edit[20] = "";
  if (snap.editId) paramText(snap.editId, snap.editIdx, edit, sizeof(edit));

  Hash hh;
  hh.add(snap.sel).str(v.label).str(v.name).add(v.color).add(v.on).add(d.solo).add(flash)
    .add(snap.editId).add(snap.editIdx).str(edit);
  Hash hl;
  hl.add(snap.sel).q(v.fader, 1023).add(v.haveFader).q(v.meter, 2).add(v.color)
    .add(snap.pickedUp).add(snap.faderTouched).q(snap.pickedUp ? 0 : snap.faderPos, 200)
    .add(editing(P_FADER));
  Hash hd;
  hd.add(dh).q(v.meter, 0.5f);  // the live dot on the mini curves

  Job jobs[] = {
    {&pHeader, hh.h, drawHeader}, {&pLevel, hl.h, drawLevel}, {&pEq, dh, drawEqPanel},
    {&pDyn, hd.h, drawDynPanel}, {&pPreamp, dh, drawPreamp}, {&pSends, dh, drawSends},
    {&pStrips, stripsHash(), drawStrips},
  };
  runJobs(jobs, 7);
}

void drawPage() {
  const StripView &v = snap.s[snap.sel];
  uint32_t dh = detailHash();
  Hash ht;
  ht.add(page).add(snap.sel).str(v.label).str(v.name).add(v.color).add(snap.d.eqOn)
    .add(snap.d.gateOn).add(snap.d.dynOn).add(snap.d.insOn);
  Hash ha, hb;
  ha.add(dh).add(page).add(pageBand).add(snap.mi.fxType);
  for (const Detail &x : snap.dAll) ha.add(x.fxslot).add(x.insOn);
  hb.add(dh).add(page).add(pageBand);
  void (*drawA)(LGFX_Sprite &) = nullptr;
  void (*drawB)(LGFX_Sprite &) = nullptr;
  switch (page) {
    case PG_EQ:     drawA = drawEqPageA; drawB = drawEqPageB; break;
    case PG_GATE:
    case PG_COMP:   drawA = drawDynPageA; drawB = drawDynPageB;
                    ha.q(v.meter, 1); hb.q(v.meter, 0.5f); break;
    case PG_SENDS:  drawA = drawSendsPage; break;
    case PG_INSERT: drawA = drawInsertPage; break;
    case PG_PREAMP: drawA = drawPreampPageA; drawB = drawPreampPageB; break;
    default: return;
  }
  Job jobs[] = {
    {&pPgTitle, ht.h, drawPageTitle}, {&pPgA, ha.h, drawA},
    {&pPgB, hb.h, drawB ? drawB : drawPageTitle}, {&pStrips, stripsHash(), drawStrips},
  };
  runJobs(jobs, drawB ? 4 : 2);
  if (!drawB) runJobs(jobs + 3, 1);
}

void drawHome() {
  Job jobs[] = {
    {&pHomeTabs, Hash().add(homeTab).add(snap.bank).add(snap.fbConnected).h, drawHomeTabs},
    {&pHomeBody, homeBodyHash(), drawHomeBody},
  };
  runJobs(jobs, 2);
}

void uiTask(void *) {
  static uint32_t fieldHash = 0;
  for (;;) {
    uint32_t start = millis();
    takeSnapshot();
    View v = pickView();
    static Page shownPage = PG_NONE;
    bool changed = v != view || (v == V_PAGE && page != shownPage);
    view = v;
    if (changed) {
      viewHash = 0;
      for (Panel *p : PANELS) { p->hash = 0; p->nz = 0; }
      if (view == V_PAGE) { shownPage = page; layoutPage(page); }
      if (view == V_MAIN || view == V_PAGE || view == V_HOME) tft.fillScreen(C_BG);
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
      case V_PAGE:
        drawPage();
        break;
      case V_HOME:
        drawHome();
        break;
    }

    handleTouch();
    if (view == V_MAIN || view == V_PAGE || view == V_HOME) drawTapMarker();
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
    if (p->w && p->h) p->spr.createSprite(p->w, p->h);
    p->hash = 0;
  }
  xTaskCreatePinnedToCore(uiTask, "ui", 8192, nullptr, 1, nullptr, 0);
}
