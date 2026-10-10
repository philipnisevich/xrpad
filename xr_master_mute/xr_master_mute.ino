// XR18 master mute button — LCKFB ESP32-S3 core board (N8R8) over Wi-Fi/OSC.
//
// Button: GPIO5 to GND (internal pull-up, no resistor needed).
// Mute LED: onboard LED on GPIO48 (active low: anode to 3V3, cathode to GPIO48)
//   fast blink = connecting / looking for mixer
//   on         = master muted
//   off        = master live

#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

const uint8_t  BUTTON_PIN   = 5;
const uint8_t  LED_PIN      = 48;
const uint16_t XR_PORT      = 10024;
const uint32_t XREMOTE_MS   = 8000;   // mixer drops subscriptions after 10 s
const uint32_t TIMEOUT_MS   = 20000;  // no packets for this long = lost mixer
const uint32_t DEBOUNCE_MS  = 30;
const uint32_t RETRY_MS     = 250;    // resend a press the mixer hasn't confirmed
const uint8_t  MAX_RETRIES  = 6;

WiFiUDP udp;
IPAddress mixerIp;
bool haveMixer = false;
int masterOn = -1;  // -1 unknown, 0 muted, 1 live
int pending = -1;   // value we sent and are waiting for the mixer to confirm
uint32_t pendingAt = 0, pressedAt = 0;
uint8_t retries = 0;
uint32_t lastRx = 0, lastXremote = 0, lastProbe = 0;

// ---------- OSC encoding ----------

size_t padTo4(size_t n) { return (n + 4) & ~3u; }  // includes the null terminator

size_t oscString(uint8_t *buf, const char *s) {
  size_t len = strlen(s), padded = padTo4(len);
  memset(buf, 0, padded);
  memcpy(buf, s, len);
  return padded;
}

void sendOsc(IPAddress ip, const char *address, const int32_t *arg = nullptr) {
  uint8_t buf[64];
  size_t n = oscString(buf, address);
  if (arg) {
    n += oscString(buf + n, ",i");
    uint32_t v = (uint32_t)*arg;
    buf[n++] = v >> 24; buf[n++] = v >> 16; buf[n++] = v >> 8; buf[n++] = v;
  }
  udp.beginPacket(ip, XR_PORT);
  udp.write(buf, n);
  udp.endPacket();
}

// ---------- OSC decoding ----------

void handlePacket(const uint8_t *buf, size_t len, IPAddress from) {
  if (len < 4) return;
  const char *address = (const char *)buf;
  size_t pos = padTo4(strnlen(address, len));
  if (pos >= len || buf[pos] != ',') return;
  const char *tags = (const char *)buf + pos;
  pos += padTo4(strnlen(tags, len - pos));

  if (strcmp(address, "/xinfo") == 0 && !haveMixer) {
    // Reply args: ip, name, model, firmware
    const char *name = pos < len ? (const char *)buf + pos + padTo4(strnlen((const char *)buf + pos, len - pos)) : "";
    mixerIp = from;
    haveMixer = true;
    Serial.printf("Found mixer \"%s\" at %s\n", name, from.toString().c_str());
    lastXremote = 0;  // subscribe immediately
  } else if (strcmp(address, "/lr/mix/on") == 0 && strcmp(tags, ",i") == 0 && pos + 4 <= len) {
    int32_t v = (int32_t)((uint32_t)buf[pos] << 24 | (uint32_t)buf[pos + 1] << 16 |
                          (uint32_t)buf[pos + 2] << 8 | buf[pos + 3]);
    int value = v ? 1 : 0;
    if (pending >= 0) {
      // Replies to earlier queries can arrive after a newer press; ignore
      // anything that doesn't match what we just sent.
      if (value != pending) {
        Serial.printf("[%lu] ignoring stale reply (%s)\n", millis(), value ? "LIVE" : "MUTED");
        return;
      }
      Serial.printf("[%lu] mixer confirmed %s in %lu ms (%u retries)\n", millis(),
                    value ? "LIVE" : "MUTED", millis() - pressedAt, retries);
      pending = -1;
    } else if (value != masterOn) {
      Serial.printf("[%lu] master is %s\n", millis(), value ? "LIVE" : "MUTED");
    }
    masterOn = value;
  }
}

void sendMute(int32_t value) {
  sendOsc(mixerIp, "/lr/mix/on", &value);
  sendOsc(mixerIp, "/lr/mix/on");  // ask the mixer to confirm
  pendingAt = millis();
}

// ---------- status LED ----------

void updateLed() {
  if (WiFi.status() != WL_CONNECTED || !haveMixer || masterOn < 0) {
    digitalWrite(LED_PIN, (millis() / 150) % 2);
  } else {
    digitalWrite(LED_PIN, masterOn ? HIGH : LOW);  // LOW = lit = muted
  }
}

// ---------- button ----------

bool buttonPressed() {
  static bool stable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  bool r = digitalRead(BUTTON_PIN);
  if (r != lastRead) { lastRead = r; changedAt = millis(); }
  if (millis() - changedAt > DEBOUNCE_MS && r != stable) {
    stable = r;
    return stable == LOW;  // fire on press, not release
  }
  return false;
}

// ---------- main ----------

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // lower latency
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    updateLed();
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
      sendOsc(mixerIp, "/xremote");    // subscribe to changes from apps/scenes
      sendOsc(mixerIp, "/lr/mix/on");  // ask for current state (also a keepalive)
    }
    if (lastRx && now - lastRx > TIMEOUT_MS) {
      Serial.println("Lost contact with mixer");
      masterOn = -1;
      lastRx = 0;
      if (!strlen(XR18_IP)) haveMixer = false;  // rediscover
    }
  }

  if (buttonPressed() && haveMixer && masterOn >= 0) {
    int next = masterOn ? 0 : 1;
    masterOn = pending = next;  // update LED right away
    pressedAt = now;
    retries = 0;
    sendMute(next);
    Serial.printf("[%lu] button: master -> %s\n", now, next ? "LIVE" : "MUTED");
  }

  if (pending >= 0 && millis() - pendingAt > RETRY_MS) {
    if (retries < MAX_RETRIES) {
      retries++;
      Serial.printf("[%lu] no confirmation, resending (%u)\n", millis(), retries);
      sendMute(pending);
    } else {
      Serial.printf("[%lu] mixer never confirmed, resyncing\n", millis());
      pending = -1;
      sendOsc(mixerIp, "/lr/mix/on");
    }
  }

  updateLed();

  static uint32_t lastLoop = 0;
  uint32_t loopEnd = millis();
  if (lastLoop && loopEnd - lastLoop > 50) {
    Serial.printf("[%lu] loop stalled %lu ms\n", loopEnd, loopEnd - lastLoop);
  }
  lastLoop = loopEnd;
  delay(2);
}
