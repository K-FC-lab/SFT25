#define BLYNK_TEMPLATE_ID "TMPL695REKxJi"
#define BLYNK_TEMPLATE_NAME "JELATECH "
#define BLYNK_AUTH_TOKEN "CIqYOlzbcLOy4PcP7VW8slcMi_rcfPQQ"

//ESP_Mesin2
#include <Arduino.h>
#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


char ssid[] = "LabKom-1";
char pass[] = "00001111";

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

#define STATUS     V3 
#define STARTSTOP  V4
#define SERVO_LED  V5
#define SERVO_GRAM V6
#define OIL_LED    V7
#define SUHU       V8
#define MOTOR_LED  V9
#define SOL_BAWAH  V10
#define STATUS_MODE V11
#define COUNTDOWN  V12
#define PENGERAS_LED V13
#define PENGERAS_GRAM V15
#define HEATER     V14

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
String statusMode = "LILIN";   // "LILIN" atau "OIL SOLIDIFIER"(Default LILIN)

// Berat & bukaan
int beratMinyak = 100;        // gram(default)
int servoAngleBase = 90;      // sudut dasar bukaan(perlu set up)
int servoOpenTimeMs = 100;    // durasi bukaan bahan (ms)(perlu set up sesuai takaran)

// Durasi proses
int pumpEODurationMs      = 500;        // ms
int mixingDurationMsBase  = 1*10*1000;  // default 5 menit (ms)
int pumpOutDurationMs     = 300;        // ms
int stepMoveDelayMs       = 1000;       // jeda setelah putar servo wadah (ms)

// Ambang suhu
float T_TIMER_START = 70.0;  // mulai hitung 5 menit
float T_OFF         = 80.0;  // heater OFF
float T_ON_HYST     = 75.0;  // heater ON kembali

// ==================== RTOS Handle ====================
TaskHandle_t TaskProcessHandle;

// ==================== Global Runtime ====================
volatile float gTempC = NAN;

// ==================== Helper ====================
void relayOn(int pin)   { digitalWrite(pin, LOW);  }
void relayOff(int pin)  { digitalWrite(pin, HIGH); }


// void pulseRelay(int pin, int duration_ms) {
//   relayOn(pin);
//   vTaskDelay(duration_ms / portTICK_PERIOD_MS);
//   relayOff(pin);
// }

void pulseRelay(int pin, int duration_ms, int vpin = -1) {
  relayOn(pin);
  if (vpin != -1) {
    Blynk.virtualWrite(vpin, digitalRead(pin) == LOW ? 1 : 0);
  }

  vTaskDelay(duration_ms / portTICK_PERIOD_MS);

  relayOff(pin);
  if (vpin != -1) {
    Blynk.virtualWrite(vpin, digitalRead(pin) == LOW ? 1 : 0);
  }
}

void servoPulse(Servo &s, int angle, int duration_ms, int vpin = -1) {
  //Hidupkan LED hanya jika mode sesuai
  if (vpin != -1) {
    Blynk.virtualWrite(vpin, 1); // LED ON
  }
  
  s.write(angle);
  vTaskDelay(duration_ms / portTICK_PERIOD_MS);
  s.write(0);

  //Matikan LED setelah servo selesai
  if (vpin != -1) {
    Blynk.virtualWrite(vpin, 0); // LED OFF
    // Blynk.run(); // pastikan sinkronisasi
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

// // Heater + Mixer: kontrol suhu + 5 menit mulai saat suhu >= 70C
void runHeaterMixerPhase() {
  relayOn(R1_Mixer);  // Mixer ON

  bool timerStarted = false;
  uint32_t tStart = 0;

  for (;;) {
    readTemperature();
    float tC = gTempC;
      // kirim status awal ke Blynk
    Blynk.virtualWrite(STATUS_MODE, "Sedang Proses");

    applyHeaterControl(tC);

    if (!timerStarted && !isnan(tC) && tC >= T_TIMER_START) {
      timerStarted = true;
      tStart = xTaskGetTickCount();
      Serial.println("[HeaterMixer] Timer start @ >=70C");
    }

    if (timerStarted) {
      uint32_t elapsed = (xTaskGetTickCount() - tStart) * portTICK_PERIOD_MS;
      if (elapsed >= (uint32_t)mixingDurationMsBase) {
        Serial.println("[HeaterMixer] Durasi selesai");
        break;
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  relayOff(R2_Heater);
  relayOff(R1_Mixer);
    // kirim status selesai ke Blynk
  Blynk.virtualWrite(STATUS_MODE, "Selesai");
}
 
// Penuangan (tiap 45 derajat dan perlu set up)
void doPouringSequence() {
  for (int angle = 0; angle <= 135; angle += 45) {
    Serial.printf("[Pour] angle=%d deg\n", angle);
    pulseRelay(R4_PumpOut, pumpOutDurationMs, SOL_BAWAH);
    servo3.write(angle);
    vTaskDelay(stepMoveDelayMs / portTICK_PERIOD_MS);
  }
  servo3.write(0);
}

// ==================== TASK PROSES ====================
void TaskProcess(void *pvParameters) {
  for (;;) {
    if (startProcess) {
      // LED -> Working
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, LOW);

      Serial.println("=== Proses Dimulai ===");
      Blynk.virtualWrite(STATUS_MODE, "Sedang Proses");

      if (statusMode == "LILIN") {
        servoPulse(servo1, servoAngleBase, servoOpenTimeMs, SERVO_LED);
        pulseRelay(R3_PumpEO, pumpEODurationMs, OIL_LED);
        runHeaterMixerPhase();
        doPouringSequence();
      }
      else if (statusMode == "OIL SOLIDIFIER") {
        servoPulse(servo2, servoAngleBase, servoOpenTimeMs, PENGERAS_LED);
        runHeaterMixerPhase();
        doPouringSequence();
      }
      Serial.println("=== Proses Selesai ===");
      Blynk.virtualWrite(STATUS_MODE, "Selesai");

      // LED -> Done selama 10 detik
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      vTaskDelay(10000 / portTICK_PERIOD_MS);

      // Kembali ke Idle
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      
        // kirim status awal ke Blynk
  Blynk.virtualWrite(STATUS_MODE, "Mesin Siap");
      startProcess = false;
    }
  // kirim status awal ke Blynk
    Blynk.virtualWrite(STATUS_MODE, "Mesin Siap");

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ================== BLYNK ==================
// Handler untuk menerima mode status dari Blynk (V3)
BLYNK_WRITE(STATUS) {
  statusMode = param.asStr();   // baca string
  Serial.print("[BLYNK] Status mode diterima: ");
  Serial.println(statusMode);
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

void sendToBlynk() {
  // kirim suhu ke widget di Blynk
  readTemperature();  // pastikan baca dulu
  Blynk.virtualWrite(SUHU, gTempC);

  // kirim status relay/LED
  Blynk.virtualWrite(MOTOR_LED, digitalRead(R1_Mixer) == LOW ? 1 : 0);
  Blynk.virtualWrite(HEATER,    digitalRead(R2_Heater) == LOW ? 1 : 0);

  Blynk.virtualWrite(SERVO_GRAM, beratMinyak);
  Blynk.virtualWrite(PENGERAS_GRAM, beratMinyak);

  if (statusMode == "OIL SOLIDIFIER") {
    Blynk.virtualWrite(PENGERAS_LED, 1);
  } else {
    Blynk.virtualWrite(PENGERAS_LED, 0);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  pinMode(R1_Mixer, OUTPUT);
  pinMode(R2_Heater, OUTPUT);
  pinMode(R3_PumpEO, OUTPUT);
  pinMode(R4_PumpOut, OUTPUT);
  relayOff(R1_Mixer);
  relayOff(R2_Heater);
  relayOff(R3_PumpEO);
  relayOff(R4_PumpOut);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  digitalWrite(led1, HIGH); // default: Idle
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);

  servo1.attach(Ser1_Parafin);
  servo2.attach(Ser2_Pengeras);
  servo3.attach(Ser3_Wadah);

  sensors.begin();

  xTaskCreatePinnedToCore(
    TaskProcess, "TaskProcess",
    6144, NULL, 1, &TaskProcessHandle, 1
  );
  //percobaan ketika manula(tidak pakai blynk)
  Serial.println("Sistem siap. Ketik 's' untuk start, 'l' LILIN, 'o' OIL SOLIDIFIER.");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Timer kirim data tiap 1 detik
  timer.setInterval(1000L, sendToBlynk);

}

// ==================== LOOP ====================
void loop() {
  Blynk.run();
  timer.run();
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') startProcess = true;
    else if (c == 'l') statusMode = "LILIN";
    else if (c == 'o') statusMode = "OIL SOLIDIFIER";
  }

  // Monitoring suhu (idle pun tetap terbaca)
  readTemperature();
  Serial.printf("[MON] T=%.2fC\n", gTempC);
  delay(800);
}
