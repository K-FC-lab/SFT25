#define BLYNK_TEMPLATE_ID "TMPL6skCxPnti"
#define BLYNK_TEMPLATE_NAME "JELATECH "
#define BLYNK_AUTH_TOKEN "JpxHwPY0jjBDknceWz52fN3J2dZNaVgQ"

//ESP_Mesin2
#include <Arduino.h>
#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "Dawilah kost3";
char pass[] = "987654321";

BlynkTimer timer;

// ==================== Pin Definition ====================
#define led1          21
#define led2          22
#define led3          23

#define R1_Mixer      14
#define R2_Heater     27
#define R3_PumpEO     26
#define R4_PumpOut    25
#define Ser1_Parafin  13
#define Ser2_Pengeras 12
#define Ser3_Wadah    33

// Virtual pins
#define STATUS        V3 
#define STARTSTOP     V4
#define SERVO_LED     V5
#define SERVO_GRAM    V6
#define OIL_LED       V7
#define SUHU          V8
#define MOTOR_LED     V9
#define SOL_BAWAH     V10
#define STATUS_MODE   V11
#define COUNTDOWN     V12
#define PENGERAS_LED  V13
#define PENGERAS_GRAM V15
#define HEATER        V14

// ==================== Sensor Suhu ====================
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// ==================== Servo ====================
Servo servo1;  // Parafin
Servo servo2;  // Pengeras
Servo servo3;  // Wadah

// ==================== Variabel Adjustable ====================
volatile bool startProcess = false;

// Untuk keandalan di multi-thread, gunakan enum integer untuk mode
enum Mode_t { MODE_LILIN = 0, MODE_OIL_SOLIDIFIER = 1 };
volatile Mode_t statusMode = MODE_LILIN;

// Berat & bukaan
int beratMinyak = 100;        // gram(default)
int servoAngleBase = 90;      // sudut dasar bukaan(perlu set up)
int servoOpenTimeMs = 100;    // durasi bukaan bahan (ms)(perlu set up sesuai takaran)

// Durasi proses
int pumpEODurationMs      = 100;          // ms
int mixingDurationMsBase  = 5 * 60 * 1000; // default 5 menit (ms)
int pumpOutDurationMs     = 300;          // ms
int stepMoveDelayMs       = 1000;         // jeda setelah putar servo wadah (ms)

// Ambang suhu
float T_TIMER_START = 70.0;  // mulai hitung waktu mixing
float T_OFF         = 80.0;  // heater OFF
float T_ON_HYST     = 75.0;  // heater ON kembali (hysteresis)

// ==================== RTOS Handle ====================
TaskHandle_t TaskProcessHandle;

// ==================== Global Runtime ====================
volatile float gTempC = NAN;

// Buffer status yang akan dikirimkan oleh task utama (agar hanya thread utama yang memanggil Blynk)
char statusMsg[32] = "Mesin Siap";

// ==================== Helper ====================
// Perhatikan: untuk board Anda, relay ON = LOW (aktif rendah). Sesuaikan jika berbeda.
void relayOn(int pin)   { digitalWrite(pin, LOW);  }
void relayOff(int pin)  { digitalWrite(pin, HIGH); }

void pulseRelay(int pin, int duration_ms) {
  relayOn(pin);
  vTaskDelay(duration_ms / portTICK_PERIOD_MS);
  relayOff(pin);
}

void servoPulse(Servo &s, int angle, int duration_ms, int vpin) {
  // Hidupkan indikator virtual (permintaan menyalakan LED di Blynk)
  if (vpin != -1) {
    // hanya set flag ke timer sendToBlynk, tapi panggilan cepat ke Blynk virtualWrite aman juga dari thread utama.
    Blynk.virtualWrite(vpin, 1); // LED ON (dipanggil dari thread utama atau timer)
  }
  
  s.write(angle);
  vTaskDelay(duration_ms / portTICK_PERIOD_MS);
  s.write(0);

  if (vpin != -1) {
    Blynk.virtualWrite(vpin, 0); // LED OFF
  }
}

void readTemperature() {
  sensors.requestTemperatures();
  gTempC = sensors.getTempCByIndex(0);
}

void applyHeaterControl(float tC) {
  static bool heaterIsOn = false;
  if (isnan(tC)) return;

  if (tC >= T_OFF && heaterIsOn) {
    relayOff(R2_Heater);
    heaterIsOn = false;
  } else if (tC <= T_ON_HYST && !heaterIsOn) {
    relayOn(R2_Heater);
    heaterIsOn = true;
  }
}

// Heater + Mixer: kontrol suhu + hitung durasi mixing setelah suhu >= T_TIMER_START
void runHeaterMixerPhase() {
  relayOn(R1_Mixer);  // Mixer ON
  bool timerStarted = false;
  uint32_t tStartTicks = 0;

  // Update status untuk dikirim ke Blynk oleh timer
  strncpy(statusMsg, "Sedang Proses", sizeof(statusMsg)-1);
  statusMsg[sizeof(statusMsg)-1]=0;

  for (;;) {
    readTemperature();
    float tC = gTempC;

    applyHeaterControl(tC);

    if (!timerStarted && !isnan(tC) && tC >= T_TIMER_START) {
      timerStarted = true;
      tStartTicks = xTaskGetTickCount();
      Serial.println("[HeaterMixer] Timer start @ >=70C");
    }

    if (timerStarted) {
      uint32_t elapsed = (xTaskGetTickCount() - tStartTicks) * portTICK_PERIOD_MS;
      // debug print tiap beberapa detik (opsional)
      // Serial.printf("[HeaterMixer] Suhu=%.2fC, Elapsed=%lums\n", tC, elapsed);

      if (elapsed >= (uint32_t)mixingDurationMsBase) {
        Serial.println("[HeaterMixer] Durasi selesai");
        break;
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  relayOff(R2_Heater);
  relayOff(R1_Mixer);

  strncpy(statusMsg, "Selesai", sizeof(statusMsg)-1);
  statusMsg[sizeof(statusMsg)-1]=0;
}

// Penuangan (tiap 45 derajat)
void doPouringSequence() {
  for (int angle = 0; angle <= 135; angle += 45) {
    Serial.printf("[Pour] angle=%d deg\n", angle);
    pulseRelay(R4_PumpOut, pumpOutDurationMs);
    servo3.write(angle);
    vTaskDelay(stepMoveDelayMs / portTICK_PERIOD_MS);
  }
  servo3.write(0);
}

// ==================== TASK PROSES ====================
void TaskProcess(void *pvParameters) {
  for (;;) {
    if (startProcess) {
      // LED -> Working (menyalakan LED fisik)
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, LOW);

      Serial.println("=== Proses Dimulai ===");
      strncpy(statusMsg, "Sedang Proses", sizeof(statusMsg)-1);
      statusMsg[sizeof(statusMsg)-1]=0;

      if (statusMode == MODE_LILIN) {
        servoPulse(servo1, servoAngleBase, servoOpenTimeMs, SERVO_LED);
        pulseRelay(R3_PumpEO, pumpEODurationMs);
        runHeaterMixerPhase();
        doPouringSequence();
      }
      else if (statusMode == MODE_OIL_SOLIDIFIER) {
        servoPulse(servo2, servoAngleBase, servoOpenTimeMs, PENGERAS_LED);
        runHeaterMixerPhase();
        doPouringSequence();
      }

      Serial.println("=== Proses Selesai ===");
      strncpy(statusMsg, "Selesai", sizeof(statusMsg)-1);
      statusMsg[sizeof(statusMsg)-1]=0;

      // LED -> Done selama 10 detik
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      vTaskDelay(10000 / portTICK_PERIOD_MS);

      // Kembali ke Idle
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);

      strncpy(statusMsg, "Mesin Siap", sizeof(statusMsg)-1);
      statusMsg[sizeof(statusMsg)-1]=0;

      startProcess = false;
    }

    // Di sini jangan panggil Blynk.virtualWrite agar tidak dari task lain
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ================== BLYNK ==================
// Handler untuk menerima mode status dari Blynk (V3)
BLYNK_WRITE(STATUS) {
  String s = param.asStr();
  Serial.print("[BLYNK] Status mode diterima: ");
  Serial.println(s);

  if (s.equalsIgnoreCase("LILIN")) {
    statusMode = MODE_LILIN;
  } else if (s.equalsIgnoreCase("OIL SOLIDIFIER") || s.equalsIgnoreCase("OIL_SOLIDIFIER") ) {
    statusMode = MODE_OIL_SOLIDIFIER;
  } else {
    // fallback: tetap LILIN
    statusMode = MODE_LILIN;
  }
}

// Handler untuk menerima perintah start/stop dari Blynk (V4)
BLYNK_WRITE(STARTSTOP) {
  int val = param.asInt();  // 1 = start, 0 = stop
  if (val == 1) {
    startProcess = true;
    Serial.println("[BLYNK] Perintah START diterima");
  } else {
    startProcess = false;
    Serial.println("[BLYNK] Perintah STOP diterima");
  }
}

// Fungsi ini berjalan di timer (thread utama Blynk) -> aman memanggil Blynk.virtualWrite()
void sendToBlynk() {
  // Baca suhu
  readTemperature();
  Blynk.virtualWrite(SUHU, isnan(gTempC) ? 0.0 : gTempC);

  // kirim status relay/LED (aktif rendah => LOW = 1)
  Blynk.virtualWrite(MOTOR_LED, digitalRead(R1_Mixer) == LOW ? 1 : 0);
  Blynk.virtualWrite(HEATER,    digitalRead(R2_Heater) == LOW ? 1 : 0);
  Blynk.virtualWrite(OIL_LED,   digitalRead(R3_PumpEO) == LOW ? 1 : 0);
  Blynk.virtualWrite(SOL_BAWAH, digitalRead(R4_PumpOut) == LOW ? 1 : 0);

  Blynk.virtualWrite(SERVO_GRAM, beratMinyak);
  Blynk.virtualWrite(PENGERAS_GRAM, beratMinyak);

  // kirim status proses (dari buffer)
  Blynk.virtualWrite(STATUS_MODE, statusMsg);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  // Output pins
  pinMode(R1_Mixer, OUTPUT);
  pinMode(R2_Heater, OUTPUT);
  pinMode(R3_PumpEO, OUTPUT);
  pinMode(R4_PumpOut, OUTPUT);
  relayOff(R1_Mixer);
  relayOff(R2_Heater);
  relayOff(R3_PumpEO);
  relayOff(R4_PumpOut);

  // LEDs fisik
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  digitalWrite(led1, HIGH); // default: Idle (note: LED logic terserah wiring)
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);

  // Servo attach
  servo1.attach(Ser1_Parafin);
  servo2.attach(Ser2_Pengeras);
  servo3.attach(Ser3_Wadah);

  sensors.begin();

  // Buat Task proses, pinned to core 1
  xTaskCreatePinnedToCore(
    TaskProcess, "TaskProcess",
    6144, NULL, 1, &TaskProcessHandle, 1
  );

  Serial.println("Sistem siap. Ketik 's' untuk start, 'l' LILIN, 'o' OIL SOLIDIFIER.");

  // Blynk begin (akan blocking sampai konek)
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Timer kirim data tiap 1 detik (jalankan di thread utama Blynk)
  timer.setInterval(1000L, sendToBlynk);
}

// ==================== LOOP ====================
void loop() {
  // Pastikan Blynk jalan & timer
  if (!Blynk.connected()) {
    // Coba reconnect (non-blocking)
    static unsigned long lastReconnectAttempt = 0;
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      Serial.println("Blynk disconnected, mencoba reconnect...");
      // Blynk.connect() tidak selalu tersedia tergantung versi--gunakan Blynk.begin() jika perlu
      Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    }
  }

  Blynk.run();
  timer.run();

  // Serial manual control
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') startProcess = true;
    else if (c == 'l') statusMode = MODE_LILIN;
    else if (c == 'o') statusMode = MODE_OIL_SOLIDIFIER;
  }

  // Monitoring suhu (print untuk debug)
  readTemperature();
  Serial.printf("[MON] T=%.2fC, Mode=%d, Start=%d\n", gTempC, (int)statusMode, (int)startProcess);

  delay(800);
}
