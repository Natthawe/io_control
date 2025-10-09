// =================== ESP32 I/O Bench + W5500/UDP ===================
/*Inputs: EMER_PIN=36 (INPUT + external pull-up), 
          BUMPER_PIN=26 (INPUT_PULLUP)
*/

/*
Outputs:  O1=12 RED, 
          O2=27 YEL,
          O3=13 GRN, 
          O4=22 BUZ(LED ฟ้า),
          O7=15 RELAY2(always ON), 
          O8=4 BYPASS (3/-3)
*/

/*
Safety Flow:
  - ชน (BUMPER rising) -> RED กระพริบ + BUZZER, ต้อง "กด EMER แล้วปลด" (ack) ก่อน
  - เมื่อ ack สำเร็จ -> GREEN (แม้ BUMPER ยังถูกกด)
*/

/*
UDP:
  - ฟังคำสั่งที่พอร์ต 8001, รายงาน IN/OUT ไปพอร์ต 8002 ของ REPORT_IP (หรือ peer ล่าสุด)
*/

//Note: BYPASS = ควบคุม O8 ด้วยคำสั่ง 3 (ON) / -3 (OFF)

// ===================================================================

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ==================== WiFi & OTA ====================

#include <WiFi.h>
#include <ArduinoOTA.h>

char name[32] = { 0 };
uint32_t chipId = 0;

bool ota_ready = false;

void getChipId() {
  chipId = 0;
  for (int i = 0; i < 17; i += 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  sprintf(name, "IO_%06X", chipId);

  Serial.printf("ChipID: %06X\n", chipId);
  Serial.printf("Generated Name: %s\n", name);
}

void OTA_Init() {
  ArduinoOTA.setHostname(name);
  ArduinoOTA.setPassword("zzzzzzzz");

  // optional: callbacks เพื่อ debug เพิ่มเติม
  ArduinoOTA
    .onStart([]() { Serial.println("[OTA] Start"); })
    .onEnd([]() { Serial.println("[OTA] End"); })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\n", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]\n", error);
    });

  // เรียก begin หลัง Wi-Fi AP พร้อมแล้ว
  ArduinoOTA.begin();
  ota_ready = true;
  Serial.println("[OTA] Ready");
}

void WiFi_Init() 
{
  const char* password = "11111111";

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(name, password);

  Serial.print("SSID: ");
  Serial.println(name);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  if (!ok) {
    Serial.println("softAP start FAILED");
  }
}

// ==================== W5500 SPI Pins (ESP32) ====================
#define W5500_MOSI  23
#define W5500_MISO  19
#define W5500_SCK   18
#define W5500_CS     5

// ==================== Network Config ====================
byte MAC[6];
IPAddress LOCAL_IP(10, 1, 100, 200);
const uint16_t UDP_PORT_LISTEN = 8001;
const uint16_t REPORT_PORT     = 8002;
EthernetUDP Udp;

IPAddress lastPeerIP(10, 1, 100, 100);
bool      hasPeer = false;

// --- Bench switches ---
#define UDP_SEND_ONLY_WHEN_LINK_UP   1
#define USE_STATIC_REPORT_IP         1
IPAddress REPORT_IP(10, 1, 100, 100); // ใส่ IP เครื่อง ROS 

// ==================== Inputs/Outputs ====================
#define EMER_PIN    36
#define BUMPER_PIN  26

const bool OUTPUT_ACTIVE_LOW = false;
const bool ACTIVE_LOW        = true;
const bool EMER_ACTIVE_LOW   = false; //(สลับ EMER: กด=HIGH=>active)
const bool BUMPER_ACTIVE_LOW = true;  //(เหมือนเดิม: กด=LOW=>active)

const uint32_t DEBOUNCE_MS        = 25;
const uint32_t STARTUP_GRACE_MS   = 300;
const uint32_t REPORT_INTERVAL_MS = 50;
const uint32_t LIDAR_HOLD_MS      = 0; // 0 = ค้างจนกว่าจะมีคำสั่งเคลียร์

uint32_t g_boot_ms = 0;

// O1..O8 -> index 0..7
const int OUT_PINS[8] = {
  12, // O1 RED
  27, // O2 YELLOW
  13, // O3 GREEN
  22, // O4 BUZZER (LED ฟ้า)
  21, // O5 (unused)
  14, // O6 (unused)
  15, // O7 RELAY2 (always ON)
  4   // O8 BYPASS (3/-3)
};

const int RED_IDX    = 0;
const int YELLOW_IDX = 1;
const int GREEN_IDX  = 2;
const int BUZZ_IDX   = 3;
const int RELAY2_IDX = 6;  // ENABLE
const int RELAY4_IDX = 7;  // BYPASS

// ==================== Blink Scheduler ====================
struct BlinkCfg {
  bool     enabled;
  bool     state;
  uint32_t on_ms, off_ms;
  uint32_t duration_ms;
  uint32_t start_ms, last_toggle_ms;
};
BlinkCfg g_blink[8];

static inline void writeOutputPin(int pin, bool on) {
  if (OUTPUT_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else                   digitalWrite(pin, on ? HIGH : LOW);
}
static inline void setOutputByIndex(int idx, bool on) {
  if (idx < 0 || idx >= 8) return;
  writeOutputPin(OUT_PINS[idx], on);
}
void setAllOutputs(bool on) { for (int i = 0; i < 8; i++) setOutputByIndex(i, on); }
void setDefaultOutputs()    { setAllOutputs(false); }

void startBlink(int idx, uint32_t on_ms, uint32_t off_ms, uint32_t duration_ms = 0) {
  if (idx < 0 || idx >= 8) return;
  g_blink[idx] = {true, true, on_ms, off_ms, duration_ms, millis(), millis()};
  setOutputByIndex(idx, true);
}
void stopBlink(int idx, bool force_off = true) {
  if (idx < 0 || idx >= 8) return;
  g_blink[idx] = {false,false,0,0,0,0,0};
  if (force_off) setOutputByIndex(idx, false);
}
void stopAllBlinks(bool force_off = true) { for (int i = 0; i < 8; i++) stopBlink(i, force_off); }

void tickBlinkAll() {
  uint32_t now = millis();
  for (int i = 0; i < 8; i++) {
    auto &b = g_blink[i];
    if (!b.enabled) continue;
    if (b.duration_ms > 0 && (now - b.start_ms) >= b.duration_ms) { stopBlink(i, true); continue; }
    uint32_t period = b.state ? b.on_ms : b.off_ms;
    if (now - b.last_toggle_ms >= period) {
      b.state = !b.state;
      setOutputByIndex(i, b.state);
      b.last_toggle_ms = now;
    }
  }
}

// ==================== Debounce (2 inputs) ====================
bool     lastStablePressed[2] = {false,false};
bool     lastReadPressed[2]   = {false,false};
uint32_t lastChangeMs[2]      = {0,0};

// inline bool rawPressedLevel(int pin) {
//   int lv = digitalRead(pin);
//   return ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
// }

//สลับสถานะ input
inline bool rawPressedLevel(int pin) {                       // NEW/CHANGED
  int lv = digitalRead(pin);
  bool active_low = (pin == EMER_PIN) ? EMER_ACTIVE_LOW
                                      : BUMPER_ACTIVE_LOW;
  return active_low ? (lv == LOW) : (lv == HIGH);
}


uint8_t updateInputsDebounced() {
  uint32_t now = millis();
  uint8_t mask = 0;
  const int pins[2] = { EMER_PIN, BUMPER_PIN };
  for (int i = 0; i < 2; i++) {
    bool reading = rawPressedLevel(pins[i]);
    if (reading != lastReadPressed[i]) { lastReadPressed[i] = reading; lastChangeMs[i] = now; }
    if ((now - lastChangeMs[i]) >= DEBOUNCE_MS) lastStablePressed[i] = reading;
    if (lastStablePressed[i]) mask |= (1 << i);
  }
  return mask;
}
inline bool isActive(uint8_t mask, int bit) { return (mask & (1 << bit)) != 0; }

// ==================== Link helper ====================
inline bool link_up() { return Ethernet.linkStatus() == LinkON; }

// ==================== Serial/UDP Report ====================
void sendInputsReport(uint8_t mask) {
  Serial.print(F("IN:")); Serial.println((unsigned)mask);
  if (UDP_SEND_ONLY_WHEN_LINK_UP && !link_up()) return;
  IPAddress dst = USE_STATIC_REPORT_IP ? REPORT_IP : lastPeerIP;
  if (!(USE_STATIC_REPORT_IP || hasPeer)) return;
  char buf[32];
  int n = snprintf(buf, sizeof(buf), "IN:%u\n", (unsigned)mask);
  Udp.beginPacket(dst, REPORT_PORT);
  Udp.write((uint8_t*)buf, n);
  Udp.endPacket();
}

uint8_t buildOutputsMask() {
  uint8_t m = 0;
  for (int i = 0; i < 8; ++i) {
    int lv = digitalRead(OUT_PINS[i]);
    bool on = OUTPUT_ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
    if (on) m |= (1 << i);
  }
  return m;
}

void sendOutputsReport() {
  uint8_t m = buildOutputsMask();
  Serial.print(F("OUT:")); Serial.println(m);
  if (UDP_SEND_ONLY_WHEN_LINK_UP && !link_up()) return;
  IPAddress dst = USE_STATIC_REPORT_IP ? REPORT_IP : lastPeerIP;
  if (!(USE_STATIC_REPORT_IP || hasPeer)) return;
  char buf[32];
  int n = snprintf(buf, sizeof(buf), "OUT:%u\n", (unsigned)m);
  Udp.beginPacket(dst, REPORT_PORT);
  Udp.write((uint8_t*)buf, n);
  Udp.endPacket();
}

// --- OUT ---
void maybeReportOutputs() {
  static uint32_t last_ms  = 0;
  static uint8_t  last_msk = 0xFF;
  uint32_t now = millis();
  uint8_t  m   = buildOutputsMask();
  if (m != last_msk || (now - last_ms) >= REPORT_INTERVAL_MS) {
    sendOutputsReport();
    last_msk = m;
    last_ms  = now;
  }
}
// -----------------------------------------------

void maybeReportInputs(uint8_t mask) {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms >= REPORT_INTERVAL_MS) { sendInputsReport(mask); last_ms = now; }
}

// ==================== System states ====================
bool emergency_active = false;  // EMER (36)
bool bumper_active    = false;  // BUMPER (26)

bool bumper_latched   = false;
bool ack_needed       = false;
bool ack_armed        = false;

bool     lidar_stop_active = false;
uint32_t lidar_stop_last_ms = 0;

int g_current_case = 9999;

int32_t  requested_case    = 9999;
uint32_t requested_case_ms = 0;
const uint32_t REQUEST_HOLD_MS = 0;

// ==================== Helpers ====================
void printState(const char* tag) {
  Serial.print(tag); Serial.print(" | ");
  for (int i = 0; i < 8; ++i) {
    int lv = digitalRead(OUT_PINS[i]);
    bool on = OUTPUT_ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
    Serial.print("O"); Serial.print(i + 1); Serial.print("=");
    Serial.print(on ? "ON" : "OFF");
    if (i < 7) Serial.print(", ");
  }
  Serial.println();
}

inline void preserveBypassOutput() {
  // คงสถานะ O8 (BYPASS) ตามค่าปัจจุบันของขานั้น
  int lv = digitalRead(OUT_PINS[RELAY4_IDX]);
  bool on = OUTPUT_ACTIVE_LOW ? (lv == LOW) : (lv == HIGH);
  setOutputByIndex(RELAY4_IDX, on);
}

inline void clearSignalsButKeepSafety() {
  setOutputByIndex(RED_IDX,    false);
  setOutputByIndex(YELLOW_IDX, false);
  setOutputByIndex(GREEN_IDX,  false);
  setOutputByIndex(BUZZ_IDX,   false);
  setOutputByIndex(RELAY2_IDX, true);  // O7 always ON
  preserveBypassOutput();
  setOutputByIndex(4, false); // O5
  setOutputByIndex(5, false); // O6
}

// ==================== Commands ====================
void applyBypass(bool on) {
  setOutputByIndex(RELAY4_IDX, on);
  Serial.print(F("BYPASS=")); Serial.println(on ? "ON" : "OFF");
}

void applyCommand(int32_t cmd) {
  switch (cmd) {
    case 1: { // EMER/BUMPER
      stopAllBlinks(false);
      clearSignalsButKeepSafety();
      if (!g_blink[RED_IDX].enabled) startBlink(RED_IDX, 500, 500, 0);
      setOutputByIndex(BUZZ_IDX, true);
      Serial.println(F("CMD 1: EMER/BUMPER"));
    } break;

    case 0: { // GREEN
      lidar_stop_active  = false;
      stopBlink(RED_IDX, true);
      setOutputByIndex(GREEN_IDX,  true);
      setOutputByIndex(YELLOW_IDX, false);
      setOutputByIndex(BUZZ_IDX,   false);
      setOutputByIndex(RELAY2_IDX, true);
      preserveBypassOutput();
      Serial.println(F("CMD 0: GREEN"));
    } break;

    case -1: { // Check Distance Lidar for stop (YELLOW)
      lidar_stop_active  = true;
      lidar_stop_last_ms = millis();
      stopBlink(RED_IDX, true);
      setOutputByIndex(GREEN_IDX,  false);
      setOutputByIndex(YELLOW_IDX, true);
      setOutputByIndex(BUZZ_IDX,   false);
      setOutputByIndex(RELAY2_IDX, true);
      preserveBypassOutput();
      Serial.println(F("CMD -1: LIDAR STOP"));
    } break;

    case 2: { // ALL OFF (keep O7 & O8)
      stopAllBlinks(true);
      clearSignalsButKeepSafety();
      Serial.println(F("CMD 2: ALL OFF (keep O7 & O8)"));
      sendOutputsReport();
    } break;

    case -2: { // ALL ON (O8 ตามสถานะ BYPASS เดิม)
      stopAllBlinks(false);
      setOutputByIndex(RED_IDX,    true);
      setOutputByIndex(YELLOW_IDX, true);
      setOutputByIndex(GREEN_IDX,  true);
      setOutputByIndex(BUZZ_IDX,   true);
      setOutputByIndex(RELAY2_IDX, true);
      preserveBypassOutput();
      Serial.println(F("CMD -2: ALL ON (O8=keep BYPASS)"));
      sendOutputsReport();
    } break;

    case 3:   applyBypass(true);  break;
    case -3:  applyBypass(false); break;

    default: {
      int v = (int)(cmd > 0 ? cmd : -cmd);
      // ปิด direct-map ของ O3..O8 สำหรับเลข 3/-3 เพราะใช้เป็น BYPASS แล้ว
      if (v >= 4 && v <= 8) {
        bool on = (cmd > 0);
        int idx = v - 1;
        stopBlink(idx, false);
        setOutputByIndex(idx, on);
        Serial.print(F("CMD ")); Serial.print(cmd);
        Serial.print(F(": O")); Serial.print(v);
        Serial.println(on ? F("=ON") : F("=OFF"));
      } else if (v == 3) {
        // ignore เพราะเคส 3/-3 ถูกจับด้านบนแล้ว
      } else {
        Serial.print(F("Unknown CMD: ")); Serial.println(cmd);
      }
    } break;
  }
  printState("STATE");
  maybeReportOutputs();                           // NEW/CHANGED: รายงาน OUT หลังเปลี่ยน
}

void applyCase(int32_t c) {
  switch (c) {
    case 1:  applyCommand(1);  break;
    case 0:  applyCommand(0);  break;
    case -1: applyCommand(-1); break;
    case 2:  applyCommand(2);  break;
    case -2: applyCommand(-2); break;
    default:
      stopBlink(RED_IDX, true);
      setOutputByIndex(BUZZ_IDX, false);
      setOutputByIndex(YELLOW_IDX, false);
      setOutputByIndex(GREEN_IDX, true);
      setOutputByIndex(RELAY2_IDX, true);
      preserveBypassOutput();
      break;
  }
}

// ==================== Decision ====================
// ตัดสินใจ “เฉพาะตอนเปลี่ยนเคส” -> ไม่รีเซ็ต blink
void resolveAndApply() {
  setOutputByIndex(RELAY2_IDX, true);
  preserveBypassOutput();

  if (lidar_stop_active && LIDAR_HOLD_MS > 0 &&
      (millis() - lidar_stop_last_ms) > LIDAR_HOLD_MS) {
    lidar_stop_active = false;
    setOutputByIndex(YELLOW_IDX, false);
  }

  int next_case = 0; // default GREEN
  if (emergency_active)       next_case = 1;
  else if (ack_needed)        next_case = 1;
  else if (lidar_stop_active) next_case = -1;
  else if (requested_case != 9999 &&
          (REQUEST_HOLD_MS == 0 || (millis() - requested_case_ms) <= REQUEST_HOLD_MS)) {
    next_case = requested_case;
  } else {
    next_case = 0;
  }

  if (next_case != g_current_case) {
    applyCase(next_case);
    g_current_case = next_case;
  }
}

// ==================== UDP Parser ====================
int32_t parseCmdFromPacket(int packetSize) {
  uint8_t buf[32];
  int len = Udp.read(buf, min(packetSize, (int)sizeof(buf) - 1));
  if (len <= 0) return 9999;

  if (len == 4) { int32_t v; memcpy(&v, buf, 4); return v; }

  buf[len] = 0;
  char *s = (char*)buf;
  while (*s == ' ' || *s == '\t') s++;
  while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ' || s[len-1] == '\t')) s[--len] = 0;
  return (int32_t)atoi(s);
}

// ==================== Helpers (MAC) ====================
void generateRandomMAC(byte *mac) {
  uint32_t r1 = esp_random();
  uint32_t r2 = esp_random();
  mac[0] = 0x02; // locally administered, unicast
  mac[1] = (r1 >>  0) & 0xFF;
  mac[2] = (r1 >>  8) & 0xFF;
  mac[3] = (r1 >> 16) & 0xFF;
  mac[4] = (r2 >>  0) & 0xFF;
  mac[5] = (r2 >>  8) & 0xFF;
}

// ==================== Setup/Loop ====================
void setup() {
  Serial.begin(115200);
  delay(200);

  getChipId();          // สร้าง name ให้เรียบร้อยก่อน
  WiFi_Init();          // <-- เริ่ม Wi-Fi AP ก่อน
  OTA_Init();           // <-- แล้วค่อย begin OTA (ภายในจะใช้ Wi-Fi)
  // ota_ready จะถูกตั้งค่าใน OTA_Init() ถ้า begin สำเร็จ

  Serial.println();
  Serial.println(F("=== ESP32 I/O Bench + W5500/UDP — BYPASS with 3/-3, cases 0/1/-1/2/-2 ==="));
  Serial.println(F("Pins: EMER=36 (INPUT + external pull-up), BUMPER=26 (INPUT_PULLUP)"));
  Serial.println(F("Serial/UDP cmds: 1,0,-1,2,-2, 3/-3 | IN:@50ms, OUT:@50ms"));

  // Inputs
  pinMode(EMER_PIN,   INPUT_PULLUP);
  pinMode(BUMPER_PIN, INPUT_PULLUP);

  // Outputs
  for (int i = 0; i < 8; i++) { pinMode(OUT_PINS[i], OUTPUT); g_blink[i] = {false,false,0,0,0,0,0}; }
  setDefaultOutputs();
  setOutputByIndex(RELAY2_IDX, true); // O7 Always ON

  // Ethernet/W5500
  SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);
  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);

  generateRandomMAC(MAC);
  Serial.print(F("MAC: "));
  for (int i = 0; i < 6; i++) { if (MAC[i] < 16) Serial.print('0'); Serial.print(MAC[i], HEX); if (i < 5) Serial.print(':'); }
  Serial.println();

  Ethernet.init(W5500_CS);
  Ethernet.begin(MAC, LOCAL_IP);

  delay(200);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("W5500 not found! Check wiring/power."));
    while (true) delay(1000);
  }
  if (!link_up()) {
    Serial.println(F("Ethernet link is OFF (check cable/switch)."));
  }

  Serial.print(F("IP address: ")); Serial.println(Ethernet.localIP());

  if (Udp.begin(UDP_PORT_LISTEN)) {
    Serial.print(F("Listening UDP port ")); Serial.println(UDP_PORT_LISTEN);
  } else {
    Serial.println(F("UDP begin() FAILED!"));
  }

  // first input report
  uint8_t mask = updateInputsDebounced();
  sendInputsReport(mask);     // ส่ง IN ครั้งแรก
  sendOutputsReport();        // ส่ง OUT ครั้งแรก

  g_boot_ms = millis();
  Serial.println(F("Ready."));
}

void loop() {
  if (ota_ready) {
    ArduinoOTA.handle();
  }

  tickBlinkAll();

  if (millis() - g_boot_ms >= STARTUP_GRACE_MS) {
    static bool prev_bumper = false;
    static bool prev_emer   = false;

    uint8_t mask = updateInputsDebounced();
    emergency_active = isActive(mask, 0);
    bumper_active    = isActive(mask, 1);

    // BUMPER Rising -> latch + require ack
    if (bumper_active && !prev_bumper) {
      bumper_latched = true;
      ack_needed     = true;
      ack_armed      = false;
      Serial.println(F("BUMPER trigger -> latch=true, ack_needed=true"));
    }

    // EMER press+release ack flow
    if (ack_needed) {
      if (emergency_active && !prev_emer) { ack_armed = true; }
      if (!emergency_active && prev_emer && ack_armed) {
        ack_needed = false;
        Serial.println(F("ACK complete (EMER press+release) -> can show GREEN"));
      }
    }

    prev_bumper = bumper_active;
    prev_emer   = emergency_active;

    resolveAndApply();
    maybeReportInputs(mask);      // เปิดรายงาน IN
    maybeReportOutputs();         // เปิดรายงาน OUT
  }

  // --- UDP receive ---
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    lastPeerIP = Udp.remoteIP();
    hasPeer = true;

    int32_t cmd = parseCmdFromPacket(packetSize);

    if      (cmd == 3)   { applyBypass(true);  }
    else if (cmd == -3)  { applyBypass(false); }
    else if (cmd == -1)  { lidar_stop_active = true; lidar_stop_last_ms = millis(); }
    else if (cmd == 1 || cmd == 0 || cmd == 2 || cmd == -2) {
      requested_case = cmd;
      requested_case_ms = millis();
    }
    applyCommand(cmd);
    resolveAndApply();
  }

  // --- Serial receive (mirror UDP) ---
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) {
      int32_t val = (int32_t)s.toInt();

      if      (val == 3)  { applyBypass(true);  }
      else if (val == -3) { applyBypass(false); }
      else if (val == -1) { lidar_stop_active = true; lidar_stop_last_ms = millis(); }
      else if (val == 1 || val == 0 || val == 2 || val == -2) {
        requested_case = val;
        requested_case_ms = millis();
      }

      applyCommand(val);
      resolveAndApply();
    }
  }
}
