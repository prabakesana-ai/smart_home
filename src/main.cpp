// =====================================================================
//  SMART HOME - Smart Environment (efisiensi energi)
//  Metode AI (tanpa ML/DL):
//    1) Fuzzy Logic (Sugeno)  -> skor kebutuhan pendinginan dari suhu & kelembapan
//    2) Logika Proposisi + Forward Chaining -> basis aturan untuk semua aktuator
// =====================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

#define DEVICE_ID   "rumah-demo-7f3a"
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
String T_CMD = String("smarthome/") + DEVICE_ID + "/cmd/#";

// ---------------- Pin ----------------
#define PIN_DHT      15   // Sensor 1: suhu & kelembapan
#define PIN_PIR_KM   32   // Sensor 2: gerak (kamar mandi)
#define PIN_LDR      34   // Sensor 3: cahaya (analog)
#define PIN_RELAY1   26   // Lampu ruang
#define PIN_RELAY2   27   // Lampu depan
#define PIN_RELAY3   33   // Lampu kamar mandi
#define PIN_IR       4    // Pemancar IR (AC)
#define PIN_WARN     25   // LED peringatan
#define PIN_AC_LED   18   // LED status AC
#define PIN_HOME_SW  14   // Sakelar di rumah / keluar

// Jika lampu menyala saat status OFF (terbalik), tukar dua nilai ini
#define RELAY_ON     HIGH
#define RELAY_OFF    LOW

// ---------------- Parameter ----------------
#define LDR_DARK_IS_HIGH  true   // jika Lampu Depan terbalik, ganti ke false
#define LDR_DARK_TH       2600   // kegelapan di atas ini  = GELAP
#define LDR_LIGHT_TH      1700   // kegelapan di bawah ini = TERANG

const float SKOR_PANAS_ON  = 70;  // skor fuzzy >= 70 -> Panas
const float SKOR_PANAS_OFF = 55;  // skor fuzzy <  55 -> tidak Panas lagi
const float SKOR_SEJUK_ON  = 30;  // skor fuzzy <= 30 -> Sejuk
const float SKOR_SEJUK_OFF = 45;  // skor fuzzy >  45 -> tidak Sejuk lagi

const unsigned long MOTION_TIMEOUT = 30000; // gerakan dianggap ada selama 30 dtk
const unsigned long AC_CEK_MS      = 30000; // AC dianggap gagal jika 30 dtk masih panas

DHT dht(PIN_DHT, DHT22);
IRsend irsend(PIN_IR);
WiFiClient net;
PubSubClient mqtt(net);

// ---------------- Status perangkat ----------------
bool relayState[3] = {false, false, false}; // 0 ruang, 1 depan, 2 kamar mandi
bool acState  = false;
bool isAtHome = false;
bool acHold   = false;   // true = AC sedang dikendalikan manual (MQTT)
unsigned long acOnSince = 0;
unsigned long lastMqttTry = 0;

// ---------------- Data sensor ----------------
float lastTemp = NAN, lastHum = NAN, fuzzyScore = 0;
float muT[3], muH[3];            // derajat keanggotaan suhu & kelembapan
bool  fPanas = false, fSejuk = false, fGelap = false;
bool  motionSeen = false;
unsigned long lastMotion = 0;

// =====================================================================
//  1) FUZZY LOGIC (model Sugeno orde nol)
//     Masukan : suhu (Sejuk, Hangat, Panas) & kelembapan (Kering, Normal, Lembap)
//     Keluaran: skor kebutuhan pendinginan 0-100
// =====================================================================
float trapmf(float x, float a, float b, float c, float d) {
  if (x < a || x > d) return 0;
  if (x >= b && x <= c) return 1;
  if (x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

float fuzzyInferensi(float t, float h) {
  // Fuzzifikasi
  muT[0] = trapmf(t, -40, -40, 24, 27);  // Sejuk
  muT[1] = trapmf(t, 25, 28, 28, 31);    // Hangat
  muT[2] = trapmf(t, 29, 32, 80, 80);    // Panas
  muH[0] = trapmf(h, 0, 0, 35, 50);      // Kering
  muH[1] = trapmf(h, 40, 55, 55, 70);    // Normal
  muH[2] = trapmf(h, 60, 75, 100, 100);  // Lembap

  // Basis aturan fuzzy: kekuatan = min (operator AND), keluaran konstanta
  float w[7], z[7];
  w[0] = muT[0];                  z[0] = 10;   // Sejuk              -> Rendah
  w[1] = fminf(muT[1], muH[0]);   z[1] = 20;   // Hangat & Kering    -> Rendah
  w[2] = fminf(muT[1], muH[1]);   z[2] = 50;   // Hangat & Normal    -> Sedang
  w[3] = fminf(muT[1], muH[2]);   z[3] = 70;   // Hangat & Lembap    -> Sedang-Tinggi
  w[4] = fminf(muT[2], muH[0]);   z[4] = 80;   // Panas  & Kering    -> Tinggi
  w[5] = fminf(muT[2], muH[1]);   z[5] = 90;   // Panas  & Normal    -> Tinggi
  w[6] = fminf(muT[2], muH[2]);   z[6] = 100;  // Panas  & Lembap    -> Sangat Tinggi

  // Defuzzifikasi: rata-rata terbobot
  float sw = 0, swz = 0;
  for (int i = 0; i < 7; i++) { sw += w[i]; swz += w[i] * z[i]; }
  return (sw > 0) ? swz / sw : 0;
}

// =====================================================================
//  2) LOGIKA PROPOSISI + FORWARD CHAINING
//     Fakta dasar  : D, G, Gelap, Panas, Sejuk, AC, Gagal
//     Fakta turunan: dibuktikan oleh aturan (closed-world: salah jika tidak terbukti)
// =====================================================================
enum Fact {
  F_D, F_G, F_GELAP, F_PANAS, F_SEJUK, F_AC, F_GAGAL,              // fakta dasar
  F_LAMPU_RUANG, F_LAMPU_DEPAN, F_LAMPU_KM,                        // turunan
  F_BUTUH_AC, F_AC_ON, F_AC_OFF, F_PERINGATAN,
  F_COUNT
};
const int F_DERIVED_START = F_LAMPU_RUANG;

const char* factName[F_COUNT] = {
  "D", "G", "Gelap", "Panas", "Sejuk", "AC", "Gagal",
  "LampuRuang", "LampuDepan", "LampuKM", "ButuhAC", "ACOn", "ACOff", "Peringatan"
};

struct Rule {
  const char* nama;
  int  n;            // jumlah anteseden
  int  ante[3];
  bool neg[3];       // true = anteseden dinegasikan
  int  cons;         // konsekuen
};

const Rule rules[] = {
  {"R1: D -> LampuRuang",                  1, {F_D, 0, 0},            {false, false, false}, F_LAMPU_RUANG},
  {"R2: D ^ G -> LampuKM",                 2, {F_D, F_G, 0},          {false, false, false}, F_LAMPU_KM},
  {"R3: Gelap -> LampuDepan",              1, {F_GELAP, 0, 0},        {false, false, false}, F_LAMPU_DEPAN},
  {"R4: D ^ Panas -> ButuhAC",             2, {F_D, F_PANAS, 0},      {false, false, false}, F_BUTUH_AC},
  {"R5: ButuhAC ^ ~AC -> ACOn",            2, {F_BUTUH_AC, F_AC, 0},  {false, true,  false}, F_AC_ON},
  {"R6: ~D ^ AC -> ACOff",                 2, {F_D, F_AC, 0},         {true,  false, false}, F_AC_OFF},
  {"R7: D ^ Sejuk ^ AC -> ACOff",          3, {F_D, F_SEJUK, F_AC},   {false, false, false}, F_AC_OFF},
  {"R8: AC ^ Gagal -> Peringatan",         2, {F_AC, F_GAGAL, 0},     {false, false, false}, F_PERINGATAN},
};
const int N_RULES = sizeof(rules) / sizeof(rules[0]);

bool facts[F_COUNT];      // fakta saat ini
bool prevFacts[F_COUNT];  // untuk mendeteksi perubahan
bool lastOut[F_COUNT];    // kesimpulan terakhir yang sudah dieksekusi

bool anteMatch(const Rule& r) {
  for (int k = 0; k < r.n; k++) {
    bool v = facts[r.ante[k]];
    if (r.neg[k]) v = !v;
    if (!v) return false;
  }
  return true;
}

void forwardChain() {
  for (int i = F_DERIVED_START; i < F_COUNT; i++) facts[i] = false;
  bool changed = true;
  int pass = 0;
  while (changed && pass < 6) {
    changed = false; pass++;
    for (int i = 0; i < N_RULES; i++) {
      if (!facts[rules[i].cons] && anteMatch(rules[i])) {
        facts[rules[i].cons] = true;
        changed = true;
      }
    }
  }
}

void updateBaseFacts() {
  facts[F_D]     = isAtHome;
  facts[F_G]     = motionSeen && (millis() - lastMotion < MOTION_TIMEOUT);
  facts[F_GELAP] = fGelap;
  facts[F_PANAS] = fPanas;
  facts[F_SEJUK] = fSejuk;
  facts[F_AC]    = acState;
  facts[F_GAGAL] = acState && fPanas && (millis() - acOnSince > AC_CEK_MS);
}

// =====================================================================
//  Aktuator & tampilan
// =====================================================================
void applyOutputs() {
  digitalWrite(PIN_RELAY1, relayState[0] ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_RELAY2, relayState[1] ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_RELAY3, relayState[2] ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_AC_LED, acState ? HIGH : LOW);
}

void printFullSystemStatus() {
  Serial.println("\n------------------------------------------------");
  Serial.println("📊 [MONITORING STATUS PERANGKAT]");
  Serial.printf("💡 Lampu Ruang       : %s\n", relayState[0] ? "🟢 MENYALA (ON)" : "🔴 MATI (OFF)");
  Serial.printf("💡 Lampu Depan       : %s\n", relayState[1] ? "🟢 MENYALA (ON)" : "🔴 MATI (OFF)");
  Serial.printf("🚿 Lampu Kamar Mandi : %s\n", relayState[2] ? "🟢 MENYALA (ON)" : "🔴 MATI (OFF)");
  Serial.printf("❄️ Status AC (Biru)   : %s\n", acState ? "🟢 MENYALA (ON)" : "🔴 MATI (OFF)");
  Serial.printf("⚠️ Peringatan         : %s\n", facts[F_PERINGATAN] ? "🟠 MENYALA" : "⚪ MATI");
  Serial.printf("🏠 Mode Keberadaan   : %s\n", isAtHome ? "DI RUMAH (Sakelar Kiri)" : "DI KANTOR / KELUAR (Sakelar Kanan)");
  Serial.printf("🌗 Cahaya            : %s\n", fGelap ? "GELAP" : "TERANG");
  Serial.printf("🌡️ Fuzzy             : T=%.1f C, H=%.0f %% -> skor %.1f\n", lastTemp, lastHum, fuzzyScore);
  Serial.println("------------------------------------------------\n");
}

void setRelay(int idx, bool on) {
  relayState[idx] = on;
  applyOutputs();
  printFullSystemStatus();
}

void triggerIR_AC(const String& cmd) {
  if (cmd == "AC_ON" || cmd == "AC_RETRY") {
    for (int i = 0; i < 3; i++) { digitalWrite(PIN_IR, HIGH); delay(50); digitalWrite(PIN_IR, LOW); delay(50); }
    irsend.sendNEC(0x20DF10EF, 32);
    acState = true;
    acOnSince = millis();
    applyOutputs();
    Serial.println("[IR] Sinyal IR AC_ON terkirim -> LED Biru AC HIGH!");
  } else if (cmd == "AC_OFF") {
    for (int i = 0; i < 3; i++) { digitalWrite(PIN_IR, HIGH); delay(50); digitalWrite(PIN_IR, LOW); delay(50); }
    irsend.sendNEC(0x20DF906F, 32);
    acState = false;
    applyOutputs();
    Serial.println("[IR] Sinyal IR AC_OFF terkirim -> LED Biru AC LOW!");
  }
  printFullSystemStatus();
}

// Eksekusi kesimpulan. Aksi hanya dijalankan saat kesimpulan BERUBAH, sehingga
// perintah manual (MQTT) tidak langsung ditimpa oleh otomatisasi.
void applyConclusions() {
  if (facts[F_LAMPU_RUANG] != lastOut[F_LAMPU_RUANG]) { lastOut[F_LAMPU_RUANG] = facts[F_LAMPU_RUANG]; setRelay(0, facts[F_LAMPU_RUANG]); }
  if (facts[F_LAMPU_DEPAN] != lastOut[F_LAMPU_DEPAN]) { lastOut[F_LAMPU_DEPAN] = facts[F_LAMPU_DEPAN]; setRelay(1, facts[F_LAMPU_DEPAN]); }
  if (facts[F_LAMPU_KM]    != lastOut[F_LAMPU_KM])    { lastOut[F_LAMPU_KM]    = facts[F_LAMPU_KM];    setRelay(2, facts[F_LAMPU_KM]); }

  if (facts[F_PERINGATAN] != lastOut[F_PERINGATAN]) {
    lastOut[F_PERINGATAN] = facts[F_PERINGATAN];
    digitalWrite(PIN_WARN, facts[F_PERINGATAN] ? HIGH : LOW);
    Serial.println(facts[F_PERINGATAN] ? "⚠️ [PERINGATAN] AC menyala tapi ruangan masih panas!" : "✅ [PERINGATAN] Padam.");
  }

  if (!acHold) {
    if (facts[F_AC_ON]  && !lastOut[F_AC_ON])  triggerIR_AC("AC_ON");
    if (facts[F_AC_OFF] && !lastOut[F_AC_OFF]) triggerIR_AC("AC_OFF");
  }
  lastOut[F_AC_ON]  = facts[F_AC_ON];
  lastOut[F_AC_OFF] = facts[F_AC_OFF];
}

// Cetak fakta dan aturan yang terpicu setiap ada perubahan
void logInference() {
  if (memcmp(facts, prevFacts, sizeof(facts)) == 0) return;
  memcpy(prevFacts, facts, sizeof(facts));
  Serial.print("\n🧠 [INFERENSI] Fakta: ");
  for (int i = 0; i < F_DERIVED_START; i++) Serial.printf("%s=%d ", factName[i], facts[i] ? 1 : 0);
  Serial.println();
  Serial.print("   Terbukti: ");
  bool any = false;
  for (int i = F_DERIVED_START; i < F_COUNT; i++) if (facts[i]) { Serial.printf("%s ", factName[i]); any = true; }
  if (!any) Serial.print("(tidak ada)");
  Serial.println();
  for (int i = 0; i < N_RULES; i++) if (anteMatch(rules[i])) Serial.printf("   aturan aktif -> %s\n", rules[i].nama);
}

// =====================================================================
//  Pembacaan sensor
// =====================================================================
int ldrDarkness() {
  int raw = analogRead(PIN_LDR);
  return LDR_DARK_IS_HIGH ? raw : 4095 - raw;
}

void readSensors() {
  static unsigned long lastLdr = 0, lastDht = 0, lastLog = 0;

  // Sensor 2: PIR kamar mandi
  if (digitalRead(PIN_PIR_KM) == HIGH) { lastMotion = millis(); motionSeen = true; }

  // Sensor 3: LDR (dua batas / histeresis)
  if (millis() - lastLdr > 500) {
    lastLdr = millis();
    int darkness = ldrDarkness();
    bool nowDark = fGelap ? (darkness >= LDR_LIGHT_TH) : (darkness > LDR_DARK_TH);
    if (nowDark != fGelap) {
      fGelap = nowDark;
      Serial.printf("\n🌗 [LDR] Hari %s (nilai kegelapan %d)\n", fGelap ? "GELAP" : "TERANG", darkness);
    }
  }

  // Sensor 1: DHT22 -> fuzzy
  if (millis() - lastDht > 2000) {
    lastDht = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (isnan(t)) return;
    if (isnan(h)) h = 50;
    lastTemp = t; lastHum = h;
    fuzzyScore = fuzzyInferensi(t, h);

    bool nowPanas = fPanas ? (fuzzyScore >= SKOR_PANAS_OFF) : (fuzzyScore >= SKOR_PANAS_ON);
    bool nowSejuk = fSejuk ? (fuzzyScore <= SKOR_SEJUK_OFF) : (fuzzyScore <= SKOR_SEJUK_ON);
    if (nowPanas != fPanas || nowSejuk != fSejuk) {
      fPanas = nowPanas; fSejuk = nowSejuk;
      acHold = false;   // kondisi berubah: otomatisasi boleh mengambil alih lagi
    }
    if (millis() - lastLog > 10000 || lastLog == 0) {
      lastLog = millis();
      Serial.printf("\n🌡️ [FUZZY] T=%.1f C, H=%.0f %%\n", t, h);
      Serial.printf("   Suhu  : Sejuk=%.2f Hangat=%.2f Panas=%.2f\n", muT[0], muT[1], muT[2]);
      Serial.printf("   Lembap: Kering=%.2f Normal=%.2f Lembap=%.2f\n", muH[0], muH[1], muH[2]);
      Serial.printf("   Skor kebutuhan pendinginan = %.1f -> %s\n", fuzzyScore, fPanas ? "PANAS" : (fSejuk ? "SEJUK" : "NORMAL"));
    }
  }
}

void handlePresence() {
  bool cur = (digitalRead(PIN_HOME_SW) == HIGH);
  if (cur != isAtHome) {
    isAtHome = cur;
    acHold = false;
    Serial.println(isAtHome ? "\n🏠 [PRESENCE] Sakelar KIRI -> Tiba di Rumah!" : "\n🚗 [PRESENCE] Sakelar KANAN -> Keluar Rumah!");
  }
}

// =====================================================================
//  MQTT (kendali jarak jauh / manual)
// =====================================================================
void onMessage(char* topic, byte* payload, unsigned int len) {
  String t(topic), p;
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  Serial.printf("\n📱 [KONTROL APLIKASI / KANTOR] %s => %s\n", topic, p.c_str());

  if (t.endsWith("/cmd/relay1"))      setRelay(0, p == "ON");
  else if (t.endsWith("/cmd/relay2")) setRelay(1, p == "ON");
  else if (t.endsWith("/cmd/relay3")) setRelay(2, p == "ON");
  else if (t.endsWith("/cmd/ir"))   { acHold = true; triggerIR_AC(p); }
}

void mqttReconnect() {
  if (mqtt.connected() || millis() - lastMqttTry < 3000) return;
  lastMqttTry = millis();
  String cid = String("esp32-") + DEVICE_ID + "-" + String(random(0xffff), HEX);
  Serial.print("[MQTT] Menghubungkan ke Broker HiveMQ... ");
  if (mqtt.connect(cid.c_str())) {
    Serial.println("TERHUBUNG (OK)!");
    mqtt.subscribe(T_CMD.c_str());
  } else {
    Serial.printf("Gagal, rc=%d\n", mqtt.state());
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);
  pinMode(PIN_RELAY3, OUTPUT);
  pinMode(PIN_WARN, OUTPUT);
  pinMode(PIN_AC_LED, OUTPUT);
  pinMode(PIN_HOME_SW, INPUT_PULLUP);
  pinMode(PIN_PIR_KM, INPUT);
  pinMode(PIN_LDR, INPUT);
  digitalWrite(PIN_WARN, LOW);
  applyOutputs();

  dht.begin();
  irsend.begin();

  Serial.print("Menghubungkan WiFi");
  WiFi.begin("Wokwi-GUEST", "", 6);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.println("\nWiFi Terhubung!");

  isAtHome = (digitalRead(PIN_HOME_SW) == HIGH);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);

  delay(1000);
  printFullSystemStatus();
}

void loop() {
  mqttReconnect();
  mqtt.loop();

  handlePresence();
  readSensors();
  updateBaseFacts();
  forwardChain();
  logInference();
  applyConclusions();
}
