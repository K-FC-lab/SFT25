#define BLYNK_TEMPLATE_ID "TMPL6skCxPnti"
#define BLYNK_TEMPLATE_NAME "JELATECH "
#define BLYNK_AUTH_TOKEN "JpxHwPY0jjBDknceWz52fN3J2dZNaVgQ"

#include <Arduino.h>
#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "Galaxy A15 43E9";
char pass[] = "11223344";

BlynkTimer timer;

// ==================== Pin Definition ====================
#define led1          21   // Mesin Siap
#define led2          22   // Sedang Proses
#define led3          23   // Proses Selesai

#define R1_Mixer      14
#define R2_Heater     27
#define R3_PumpEO     26
#define R4_PumpOut    25
#define Ser1_Parafin  13
#define Ser2_Pengeras 12
#define Ser3_Wadah    33

#define STATUS       V3 
#define STARTSTOP    V4
#define SERVO_LED    V5
#define SERVO_GRAM   V6
#define OIL_LED      V7
#define SUHU         V8
#define MOTOR_LED    V9
#define SOL_BAWAH    V10
#define STATUS_MODE  V11
#define PENGERAS_LED V12
#define HEATER       V14
#define PENGERAS_GRAM V13

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
String statusMode = "LILIN";   // Default: LILIN

int beratMinyak = 100;        
int servoAngleBase = 90;      
int servoOpenTimeMs = 100;    

int pumpEODurationMs      = 100;        
int mixingDurationMsBase  = 1*10*1000;  
int pumpOutDurationMs     = 300;        
int stepMoveDelayMs       = 1000;       

float T_TIMER_START = 70.0;  
float T_OFF         = 80.0;  
float T_ON_HYST     = 75.0;  

TaskHandle_t TaskProcessHandle;
volatile float gTempC = NAN;

// ==================== Helper ====================
void relayOn(int pin)   { digitalWrite(pin, LOW);  }
void relayOff(int pin)  { digitalWrite(pin, HIGH); }

void pulseRelay(int pin, int duration_ms) {
  relayOn(pin);
  vTaskDelay(duration_ms / portTICK_PERIOD_MS);
  relayOff(pin);
}

void servoPulse(Servo &s, int angle, int duration_ms, int vpin) {
  if (vpin != -1) Blynk.virtualWrite(vpin, 1); // LED ON
  s.write(angle);
  vTaskDelay(duration_ms / portTICK_PERIOD_MS);
  s.write(0);
  if (vpin != -1) Blynk.virtualWrite(vpin, 0); // LED OFF
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

void runHeaterMixerPhase() {
  relayOn(R1_Mixer);  
  bool timerStarted = false;
  uint32_t tStart = 0;

  for (;;) {
    readTemperature();
    float tC = gTempC;
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
  Blynk.virtualWrite(STATUS_MODE, "Selesai");
}

void doPouringSequence() {
  for (int angle = 0; angle <= 135; angle += 45) {
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
      // === Sedang Proses ===
      digitalWrite(led1, LOW);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, LOW);
      Blynk.virtualWrite(STATUS_MODE, "Sedang Proses");

      Serial.println("=== Proses Dimulai ===");

      if (statusMode == "LILIN") {
        servoPulse(servo1, servoAngleBase, servoOpenTimeMs, SERVO_LED);
        pulseRelay(R3_PumpEO, pumpEODurationMs);
        runHeaterMixerPhase();
        doPouringSequence();
      }
      else if (statusMode == "OIL SOLIDIFIER") {
        servoPulse(servo2, servoAngleBase, servoOpenTimeMs, PENGERAS_LED);
        runHeaterMixerPhase();
        doPouringSequence();
      }

      Serial.println("=== Proses Selesai ===");

      // === Proses Selesai ===
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      Blynk.virtualWrite(STATUS_MODE, "Selesai");

      vTaskDelay(10000 / portTICK_PERIOD_MS);

      // === Mesin Siap lagi ===
      digitalWrite(led1, HIGH);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      Blynk.virtualWrite(STATUS_MODE, "Mesin Siap");

      startProcess = false;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ================== BLYNK ==================
BLYNK_WRITE(STATUS) {
  statusMode = param.asStr();
  Serial.print("[BLYNK] Status mode diterima: ");
  Serial.println(statusMode);
}

BLYNK_WRITE(STARTSTOP) {
  int val = param.asInt();  
  if (val == 1) {
    startProcess = true;
    Serial.println("[BLYNK] Perintah START diterima");
  } else {
    startProcess = false;
    Serial.println("[BLYNK] Perintah STOP diterima");
  }
}

void sendToBlynk() {
  readTemperature();
  Blynk.virtualWrite(SUHU, gTempC);
  Blynk.virtualWrite(MOTOR_LED, digitalRead(R1_Mixer) == LOW ? 1 : 0);
  Blynk.virtualWrite(HEATER,    digitalRead(R2_Heater) == LOW ? 1 : 0);
  Blynk.virtualWrite(OIL_LED,   digitalRead(R3_PumpEO) == LOW ? 1 : 0);
  Blynk.virtualWrite(SOL_BAWAH, digitalRead(R4_PumpOut) == LOW ? 1 : 0);
  Blynk.virtualWrite(SERVO_GRAM, beratMinyak);
  Blynk.virtualWrite(PENGERAS_GRAM, beratMinyak);
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
  digitalWrite(led1, HIGH); // Mesin Siap default
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

  Serial.println("Sistem siap. Tunggu perintah START dari Blynk.");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(1000L, sendToBlynk);
}

// ==================== LOOP ====================
void loop() {
  Blynk.run();
  timer.run();

  // Monitoring suhu saat idle
  readTemperature();
  Serial.printf("[MON] T=%.2fC\n", gTempC);
  delay(800);
}
