/*
 * SmartTrafficLights - ESP32 hardware node (Stage 9)
 *
 * Role: report real IR detectors and display commanded signal states.
 * It makes NO traffic decisions: Python's SignalBank is authoritative.
 *
 * Protocol (see smarttraffic/esp_protocol.py and hardware/README.md):
 *   frames  "$FIELD,FIELD,...*CS\n", CS = XOR of bytes between '$' and '*' (2 hex digits)
 *   ESP32 -> PC : HELLO,seq,ms,proto,device_id,boot_id,n_sensors,n_signals
 *                 SEN,seq,ms,channel,OCC|CLR     (edge, or full-state report)
 *                 HB,seq,ms                      (heartbeat, every HEARTBEAT_MS)
 *                 ACK,seq,ms,channel,STATE       (signal command applied)
 *                 ERR,seq,ms,text
 *   PC -> ESP32 : SIG,channel,STATE   STATE = RED|RED_YELLOW|GREEN|FLASHING_GREEN|YELLOW|ALL_RED|OFF
 *                 QRY                 re-send HELLO + all sensor states
 *
 * FLASHING_GREEN is blinked HERE: 1 Hz, lamp ON for the first 500 ms of each
 * second, phase starting when the FLASHING_GREEN command arrives. Re-sending the
 * same state (refresh) does not restart the phase. ALL_RED shows the red lamp.
 * If no SIG command arrives for SIGNAL_TIMEOUT_MS, every head shows the fault
 * indication (yellow blinking at 1 Hz) until commands resume.
 *
 * Library: Adafruit NeoPixel (Arduino Library Manager). Board: any ESP32.
 */
#include <Adafruit_NeoPixel.h>

// ============================ CONFIGURATION ============================
static const char *DEVICE_ID = "esp32_a";      // must match config/hardware.json
static const uint32_t BAUD = 115200;

struct SensorCfg {
  uint8_t  pin;            // ADC pin (ADC1: 32-39 recommended)
  uint16_t threshold;      // raw analogRead threshold
  bool     occupiedAbove;  // true: value > threshold means OCCUPIED
  uint16_t debounceMs;     // new state must persist this long before it is reported
};
// channel = index in this table (config/hardware.json "channel")
static const SensorCfg SENSORS[] = {
  {34, 2000, true, 20},    // channel 0: analog IR, car present when analogRead > 2000
};

struct SignalCfg {
  uint8_t pin;             // NeoPixel data pin
  uint8_t numLeds;
  uint8_t greenIdx, yellowIdx, redIdx;
  uint8_t brightness;      // 0-255
};
static const SignalCfg SIGNALS[] = {
  {13, 3, 0, 1, 2, 80},    // channel 0: 3-LED module, GREEN=0, YELLOW=1, RED=2
};

static const uint32_t HEARTBEAT_MS      = 200;
static const uint32_t SIGNAL_TIMEOUT_MS = 3000;
static const uint32_t SAMPLE_MS         = 2;
static const uint32_t FLASH_PERIOD_MS   = 1000;
static const uint32_t FLASH_ON_MS       = 500;
static const int      PROTOCOL_VERSION  = 1;
// =======================================================================

static const uint8_t N_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);
static const uint8_t N_SIGNALS = sizeof(SIGNALS) / sizeof(SIGNALS[0]);

enum SigState { S_RED, S_RED_YELLOW, S_GREEN, S_FLASHING_GREEN, S_YELLOW, S_ALL_RED, S_OFF, S_FAULT };
static const char *STATE_NAMES[] = {"RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED", "OFF"};

static Adafruit_NeoPixel *strips[N_SIGNALS];
static SigState sigState[N_SIGNALS];
static uint32_t sigSince[N_SIGNALS];
static uint32_t lastCommandMs = 0;
static bool     everCommanded = false;

static bool     sensState[N_SENSORS];        // reported state
static bool     sensCandidate[N_SENSORS];    // raw state waiting for debounce
static uint32_t sensCandidateSince[N_SENSORS];

static uint32_t seqNo = 0;
static char     bootId[9];
static uint32_t lastHeartbeat = 0, lastSample = 0;
static char     rxBuf[128];
static uint8_t  rxLen = 0;

// ------------------------------------------------------------ framing
static void sendFrame(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; ++p) cs ^= (uint8_t)*p;
  char out[140];
  snprintf(out, sizeof(out), "$%s*%02X\n", body, cs);
  Serial.print(out);
}

static void sendHello() {
  char b[100];
  snprintf(b, sizeof(b), "HELLO,%lu,%lu,%d,%s,%s,%u,%u", (unsigned long)seqNo++, (unsigned long)millis(),
           PROTOCOL_VERSION, DEVICE_ID, bootId, N_SENSORS, N_SIGNALS);
  sendFrame(b);
}

static void sendSensor(uint8_t ch, bool occ, uint32_t atMs) {
  char b[64];
  snprintf(b, sizeof(b), "SEN,%lu,%lu,%u,%s", (unsigned long)seqNo++, (unsigned long)atMs, ch, occ ? "OCC" : "CLR");
  sendFrame(b);
}

static void sendAllSensors() {
  for (uint8_t i = 0; i < N_SENSORS; ++i) sendSensor(i, sensState[i], millis());
}

static void sendError(const char *text) {
  char b[100];
  snprintf(b, sizeof(b), "ERR,%lu,%lu,%s", (unsigned long)seqNo++, (unsigned long)millis(), text);
  sendFrame(b);
}

// ------------------------------------------------------------ lamps
static int8_t lastLamps[N_SIGNALS];          // bitmask R=4 Y=2 G=1; -1 = unknown

static void setLamps(uint8_t ch, bool r, bool y, bool g) {
  int8_t mask = (r ? 4 : 0) | (y ? 2 : 0) | (g ? 1 : 0);
  if (mask == lastLamps[ch]) return;          // only drive the LEDs when something changes
  lastLamps[ch] = mask;
  Adafruit_NeoPixel *s = strips[ch];
  const SignalCfg &c = SIGNALS[ch];
  s->clear();
  if (r) s->setPixelColor(c.redIdx, s->Color(255, 0, 0));
  if (y) s->setPixelColor(c.yellowIdx, s->Color(255, 140, 0));
  if (g) s->setPixelColor(c.greenIdx, s->Color(0, 255, 0));
  s->show();
}

static void render(uint8_t ch, uint32_t now) {
  SigState st = sigState[ch];
  if (!everCommanded || now - lastCommandMs > SIGNAL_TIMEOUT_MS) st = S_FAULT;
  switch (st) {
    case S_RED: case S_ALL_RED: setLamps(ch, true, false, false); break;
    case S_RED_YELLOW:          setLamps(ch, true, true, false); break;
    case S_GREEN:               setLamps(ch, false, false, true); break;
    case S_FLASHING_GREEN:      setLamps(ch, false, false, ((now - sigSince[ch]) % FLASH_PERIOD_MS) < FLASH_ON_MS); break;
    case S_YELLOW:              setLamps(ch, false, true, false); break;
    case S_OFF:                 setLamps(ch, false, false, false); break;
    case S_FAULT:               setLamps(ch, false, (now % FLASH_PERIOD_MS) < FLASH_ON_MS, false); break;
  }
}

// ------------------------------------------------------------ commands
static bool checkFrame(char *line, char **body) {
  size_t n = strlen(line);
  if (n < 4 || line[0] != '$' || line[n - 3] != '*') return false;
  line[n - 3] = '\0';
  uint8_t cs = 0;
  for (char *p = line + 1; *p; ++p) cs ^= (uint8_t)*p;
  char hex[3];
  snprintf(hex, sizeof(hex), "%02X", cs);
  if (toupper(line[n - 2]) != hex[0] || toupper(line[n - 1]) != hex[1]) return false;
  *body = line + 1;
  return true;
}

static void handleLine(char *line) {
  char *body;
  if (!checkFrame(line, &body)) { sendError("bad_frame"); return; }
  if (strcmp(body, "QRY") == 0) { sendHello(); sendAllSensors(); return; }
  if (strncmp(body, "SIG,", 4) == 0) {
    char *chStr = body + 4;
    char *comma = strchr(chStr, ',');
    if (!comma) { sendError("bad_sig"); return; }
    *comma = '\0';
    int ch = atoi(chStr);
    const char *name = comma + 1;
    if (ch < 0 || ch >= N_SIGNALS) { sendError("bad_channel"); return; }
    int st = -1;
    for (int i = 0; i <= S_OFF; ++i) if (strcmp(name, STATE_NAMES[i]) == 0) st = i;
    if (st < 0) { sendError("bad_state"); return; }
    uint32_t now = millis();
    lastCommandMs = now;
    everCommanded = true;
    if ((SigState)st != sigState[ch]) { sigState[ch] = (SigState)st; sigSince[ch] = now; }  // refresh keeps phase
    char b[64];
    snprintf(b, sizeof(b), "ACK,%lu,%lu,%d,%s", (unsigned long)seqNo++, (unsigned long)now, ch, name);
    sendFrame(b);
    return;
  }
  sendError("unknown_cmd");
}

static void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { rxBuf[rxLen] = '\0'; if (rxLen && rxBuf[rxLen - 1] == '\r') rxBuf[rxLen - 1] = '\0'; if (rxBuf[0]) handleLine(rxBuf); rxLen = 0; }
    else if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else { rxLen = 0; sendError("line_too_long"); }
  }
}

// ------------------------------------------------------------ sensors
static bool rawOccupied(uint8_t i) {
  int v = analogRead(SENSORS[i].pin);
  return SENSORS[i].occupiedAbove ? (v > SENSORS[i].threshold) : (v < SENSORS[i].threshold);
}

static void sampleSensors(uint32_t now) {
  for (uint8_t i = 0; i < N_SENSORS; ++i) {
    bool raw = rawOccupied(i);
    if (raw != sensCandidate[i]) { sensCandidate[i] = raw; sensCandidateSince[i] = now; }
    if (sensCandidate[i] != sensState[i] && now - sensCandidateSince[i] >= SENSORS[i].debounceMs) {
      sensState[i] = sensCandidate[i];
      sendSensor(i, sensState[i], sensCandidateSince[i]);   // timestamp = first sample of the new state
    }
  }
}

// ------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(BAUD);
  snprintf(bootId, sizeof(bootId), "%08lX", (unsigned long)esp_random());
  for (uint8_t i = 0; i < N_SIGNALS; ++i) {
    strips[i] = new Adafruit_NeoPixel(SIGNALS[i].numLeds, SIGNALS[i].pin, NEO_GRB + NEO_KHZ800);
    strips[i]->begin();
    strips[i]->setBrightness(SIGNALS[i].brightness);
    sigState[i] = S_RED;
    sigSince[i] = 0;
    lastLamps[i] = -1;
  }
  analogReadResolution(12);
  uint32_t now = millis();
  for (uint8_t i = 0; i < N_SENSORS; ++i) {
    sensState[i] = sensCandidate[i] = rawOccupied(i);
    sensCandidateSince[i] = now;
  }
  sendHello();
  sendAllSensors();
}

void loop() {
  uint32_t now = millis();
  readSerial();
  if (now - lastSample >= SAMPLE_MS) { lastSample = now; sampleSensors(now); }
  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    char b[40];
    snprintf(b, sizeof(b), "HB,%lu,%lu", (unsigned long)seqNo++, (unsigned long)now);
    sendFrame(b);
  }
  for (uint8_t i = 0; i < N_SIGNALS; ++i) render(i, now);
}
