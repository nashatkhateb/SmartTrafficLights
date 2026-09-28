/*
 * SmartTrafficLights - ESP32 hardware node (protocol v2, discrete R/Y/G light)
 *
 * Role: report the two real IR detectors and set the physical traffic light
 * exactly as commanded. It makes NO traffic decisions and runs NO traffic-light
 * timing: Python's SignalBank / controller is the only timing authority.
 *
 * Verified demo hardware
 *   Sensor channel 0 = NS:D1 : digital IR obstacle module, OUT -> GPIO27, active-low
 *   Sensor channel 1 = NS:D2 : digital IR obstacle module, OUT -> GPIO26, active-low
 *                              (both: VCC -> 3.3V, GND -> GND; HIGH = clear, LOW = car)
 *   Signal channel 0 = NS    : 3-LED module, G -> GPIO25, Y -> GPIO33, R -> GPIO32,
 *                              GND -> GND; output HIGH = lamp ON
 *
 * Protocol (see smarttraffic/esp_protocol.py and hardware/README.md):
 *   frames  "$FIELD,FIELD,...*CS\n", CS = XOR of bytes between '$' and '*' (2 hex digits)
 *   ESP32 -> PC : HELLO,seq,ms,proto,device_id,boot_id,n_sensors,n_signals
 *                 SEN,seq,ms,channel,OCC|CLR     (edge, or full-state report)
 *                 HB,seq,ms                      (heartbeat, every HEARTBEAT_MS)
 *                 ACK,seq,ms,channel,STATE,LAMPS (signal command applied)
 *                 ERR,seq,ms,text
 *   PC -> ESP32 : SIG,channel,STATE,LAMPS  STATE = RED|RED_YELLOW|GREEN|FLASHING_GREEN|YELLOW|ALL_RED|OFF
 *                                          LAMPS = "RYG" as 0/1, e.g. 100 = red, 110 = red+yellow
 *                 QRY                      re-send HELLO + all sensor states
 *
 * The PC sends a SIG on every lamp change of the logical signal (FLASHING_GREEN
 * arrives as 001 / 000 at each flash edge) and refreshes it every second. The
 * lamps are applied exactly as received; the pattern must be one the state can
 * show, otherwise the command is refused (ERR bad_lamps) and nothing changes.
 *
 * Fail-safe: if no SIG arrives for SIGNAL_TIMEOUT_MS (USB unplugged, PC program
 * stopped or paused), or none has arrived since boot, the head shows the fault
 * indication: yellow blinking at 1 Hz, red and green off. Normal display resumes
 * with the next command. A sensor module that is disconnected reads HIGH through
 * the internal pull-up, i.e. CLEAR (no phantom vehicle).
 *
 * No extra library needed. Board: any ESP32 (Arduino core).
 */

// ============================ CONFIGURATION ============================
static const char *DEVICE_ID = "esp32_a";      // must match config/hardware.json
static const uint32_t BAUD = 115200;

struct SensorCfg {
  uint8_t  pin;            // digital input
  bool     activeLow;      // true: LOW = OCCUPIED (car), HIGH = CLEAR
  uint16_t debounceMs;     // new state must persist this long before it is reported
};
// channel = index in this table (config/hardware.json "channel")
static const SensorCfg SENSORS[] = {
  {27, true, 20},          // channel 0: NS:D1, digital IR, LOW = car
  {26, true, 20},          // channel 1: NS:D2, digital IR, LOW = car
};

struct SignalCfg {
  uint8_t greenPin, yellowPin, redPin;
  bool    activeHigh;      // true: output HIGH = lamp ON
};
static const SignalCfg SIGNALS[] = {
  {25, 33, 32, true},      // channel 0: NS head, G=GPIO25 Y=GPIO33 R=GPIO32
};

static const uint32_t HEARTBEAT_MS      = 200;
static const uint32_t SIGNAL_TIMEOUT_MS = 3000;
static const uint32_t SAMPLE_MS         = 2;
static const uint32_t FAULT_BLINK_PERIOD_MS = 1000;   // fail-safe indication only
static const uint32_t FAULT_BLINK_ON_MS     = 500;
static const int      PROTOCOL_VERSION  = 2;
// =======================================================================

static const uint8_t N_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);
static const uint8_t N_SIGNALS = sizeof(SIGNALS) / sizeof(SIGNALS[0]);

enum SigState { S_RED, S_RED_YELLOW, S_GREEN, S_FLASHING_GREEN, S_YELLOW, S_ALL_RED, S_OFF };
static const char *STATE_NAMES[] = {"RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED", "OFF"};
// lamp patterns ("RYG") each state may show; FLASHING_GREEN alternates 001 / 000
static const char *STATE_LAMPS_A[] = {"100", "110", "001", "001", "010", "100", "000"};
static const char *STATE_LAMPS_B[] = {"100", "110", "001", "000", "010", "100", "000"};

static bool     lampR[N_SIGNALS], lampY[N_SIGNALS], lampG[N_SIGNALS];   // last commanded lamps
static uint32_t lastCommandMs = 0;
static bool     everCommanded = false;

static bool     sensState[N_SENSORS];        // reported state (true = occupied)
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
static void writeLamp(uint8_t pin, bool on, bool activeHigh) {
  digitalWrite(pin, (on == activeHigh) ? HIGH : LOW);
}

static void setOutputs(uint8_t ch, bool r, bool y, bool g) {
  const SignalCfg &c = SIGNALS[ch];
  writeLamp(c.redPin, r, c.activeHigh);
  writeLamp(c.yellowPin, y, c.activeHigh);
  writeLamp(c.greenPin, g, c.activeHigh);
}

static void render(uint8_t ch, uint32_t now) {
  if (!everCommanded || now - lastCommandMs > SIGNAL_TIMEOUT_MS) {       // fail-safe
    setOutputs(ch, false, (now % FAULT_BLINK_PERIOD_MS) < FAULT_BLINK_ON_MS, false);
  } else {
    setOutputs(ch, lampR[ch], lampY[ch], lampG[ch]);                   // exactly as commanded
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
    char *c1 = strchr(chStr, ',');
    if (!c1) { sendError("bad_sig"); return; }
    *c1 = '\0';
    char *name = c1 + 1;
    char *c2 = strchr(name, ',');
    if (!c2) { sendError("bad_sig"); return; }
    *c2 = '\0';
    const char *lamps = c2 + 1;
    int ch = atoi(chStr);
    if (ch < 0 || ch >= N_SIGNALS) { sendError("bad_channel"); return; }
    int st = -1;
    for (int i = 0; i <= S_OFF; ++i) if (strcmp(name, STATE_NAMES[i]) == 0) st = i;
    if (st < 0) { sendError("bad_state"); return; }
    if (strcmp(lamps, STATE_LAMPS_A[st]) != 0 && strcmp(lamps, STATE_LAMPS_B[st]) != 0) {
      sendError("bad_lamps"); return;
    }
    uint32_t now = millis();
    lastCommandMs = now;
    everCommanded = true;
    lampR[ch] = lamps[0] == '1';
    lampY[ch] = lamps[1] == '1';
    lampG[ch] = lamps[2] == '1';
    render(ch, now);                                   // apply immediately
    char b[64];
    snprintf(b, sizeof(b), "ACK,%lu,%lu,%d,%s,%s", (unsigned long)seqNo++, (unsigned long)now, ch, name, lamps);
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
  int level = digitalRead(SENSORS[i].pin);
  return SENSORS[i].activeLow ? (level == LOW) : (level == HIGH);
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
    pinMode(SIGNALS[i].greenPin, OUTPUT);
    pinMode(SIGNALS[i].yellowPin, OUTPUT);
    pinMode(SIGNALS[i].redPin, OUTPUT);
    lampR[i] = lampY[i] = lampG[i] = false;
    setOutputs(i, false, false, false);
  }
  uint32_t now = millis();
  for (uint8_t i = 0; i < N_SENSORS; ++i) {
    pinMode(SENSORS[i].pin, INPUT_PULLUP);     // unplugged module reads HIGH = CLEAR
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
