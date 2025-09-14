 
#define BLYNK_TEMPLATE_ID "TMPL6f4Uf54az"
#define BLYNK_TEMPLATE_NAME "Monitoring"
#define BLYNK_AUTH_TOKEN "9lbx7h4Z1BGfKhBEfQ9sDrW2YP8FnGJI"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "LabKom-1";
char pass[] = "00001111";

BlynkTimer timer;

/* =========================================================
   MESIN 1 : Monitoring minyak jelantah
   ========================================================= */
int motorPin1 = 2;
float suhu = 0;
int beratInput = 0;
bool startStop1 = false;
bool autoMode1 = true;

// Virtual Pin Mesin 1
#define SUHU     V0
#define START1   V1
#define BERAT    V2
#define STATUS1  V3

/* =========================================================
   MESIN 2 : Pembuatan lilin dari minyak jelantah
   ========================================================= */
enum ProcState { IDLE, PROCESS, DONE };
ProcState stateMesin2 = IDLE;

unsigned long remainingSec = 20;       // durasi mixing (20 detik untuk tes)
const unsigned long MIX_TIME_SEC = 20; // default
bool startStop2 = false;

// Virtual Pin Mesin 2
#define STARTSTOP  V4
#define SERVO_LED  V5
#define SERVO_GRAM V6
#define OIL_LED    V7
#define HEATER     V8
#define MOTOR_LED  V9
#define SOL_BAWAH  V10
#define STATUS     V11
#define COUNTDOWN  V12

/* =========================================================
   FUNGSI MESIN 1
   ========================================================= */
void sendSensor1() {
  // Buat nilai random
  suhu = random(25, 60);         // suhu random 25–60 °C
  beratInput = random(0, 50);    // berat random 0–50

  int motorState = random(0, 2); // motor random
  digitalWrite(motorPin1, motorState);
  Blynk.virtualWrite(SUHU, suhu);
  Blynk.virtualWrite(BERAT, beratInput);
  Blynk.virtualWrite(MOTOR, motorState);

  // cek status
  String statusMesin;
  if (suhu < 40 && beratInput > 10 && startStop1) {
    statusMesin = "BISA DIOLAH";
  } else {
    statusMesin = "TIDAK BISA DIOLAH";
  }
  Blynk.virtualWrite(STATUS1, statusMesin);

  // ===== Serial Monitor Output Mesin 1 =====
  Serial.println("=== MESIN 1 ===");
  Serial.print("Suhu: "); Serial.println(suhu);
  Serial.print("Berat Input: "); Serial.println(beratInput);
  Serial.print("Motor: "); Serial.println(motorState);
  Serial.print("Start/Stop: "); Serial.println(startStop1);
  Serial.print("Status: "); Serial.println(statusMesin);
  Serial.println("================");
}

BLYNK_WRITE(START1) {
  int tombol = param.asInt();
  startStop1 = (tombol == 1);
  Serial.print("[M1] Start/Stop tombol: ");
  Serial.println(startStop1);
}

/* =========================================================
   FUNGSI MESIN 2
   ========================================================= */
String mmss(unsigned long s) {
  unsigned long m = s / 60;
  unsigned long sec = s % 60;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", m, sec);
  return String(buf);
}

void pushUI2() {
  // Servo gram random
  int servoGram = random(20, 31);
  bool servoOn = (stateMesin2 == PROCESS);

  // Oil LED aktif di awal
  bool oilOn = (stateMesin2 == PROCESS && (MIX_TIME_SEC - remainingSec) < 5);

  // Heater random
  int heaterVal = (stateMesin2 == PROCESS) ? random(60, 101) : 0;

  // Motor
  bool motorOn = (stateMesin2 == PROCESS);

  // Countdown
  String waktu = mmss(remainingSec);

  // Solenoid bawah aktif 3 detik terakhir
  bool solBawahOn = (stateMesin2 == PROCESS && remainingSec <= 3);

  // Status teks
  String st = "Idle";
  if (stateMesin2 == PROCESS) st = "On Process";
  else if (stateMesin2 == DONE) st = "Sudah Selesai";

  // kirim ke Blynk
  Blynk.virtualWrite(SERVO_LED, servoOn ? 255 : 0);
  Blynk.virtualWrite(SERVO_GRAM, String(servoGram) + " g");
  Blynk.virtualWrite(OIL_LED, oilOn ? 255 : 0);
  Blynk.virtualWrite(HEATER, heaterVal);
  Blynk.virtualWrite(MOTOR_LED, motorOn ? 255 : 0);
  Blynk.virtualWrite(COUNTDOWN, waktu);
  Blynk.virtualWrite(SOL_BAWAH, solBawahOn ? 255 : 0);
  Blynk.virtualWrite(STATUS, st);

  // ===== Serial Monitor Output Mesin 2 =====
  Serial.println("=== MESIN 2 ===");
  Serial.print("Servo Gram: "); Serial.println(servoGram);
  Serial.print("Servo LED: "); Serial.println(servoOn);
  Serial.print("Oil LED: "); Serial.println(oilOn);
  Serial.print("Heater: "); Serial.println(heaterVal);
  Serial.print("Motor: "); Serial.println(motorOn);
  Serial.print("Countdown: "); Serial.println(waktu);
  Serial.print("Solenoid Bawah: "); Serial.println(solBawahOn);
  Serial.print("Status: "); Serial.println(st);
  Serial.println("================");
}

void tickProcess2() {
  if (stateMesin2 == PROCESS) {
    if (remainingSec > 0) remainingSec--;
    if (remainingSec == 0) {
      stateMesin2 = DONE;
    }
    pushUI2();
  }
}

BLYNK_WRITE(STARTSTOP) {
  int val = param.asInt();
  if (val == 1) {
    stateMesin2 = PROCESS;
    remainingSec = MIX_TIME_SEC;
    startStop2 = true;
  } else {
    stateMesin2 = IDLE;
    remainingSec = MIX_TIME_SEC;
    startStop2 = false;
  }
  Serial.print("[M2] Start/Stop tombol: ");
  Serial.println(startStop2);
  pushUI2();
}

/* =========================================================
   SETUP & LOOP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  pinMode(motorPin1, OUTPUT);
  

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Mesin 1 update tiap 2 detik
  timer.setInterval(2000L, sendSensor1);

  // Mesin 2 update tiap 1 detik
  timer.setInterval(1000L, tickProcess2);

  Serial.println("=== SISTEM MONITORING LILIN & MINYAK JELANTAH SIAP ===");
}

void loop() {
  Blynk.run();
  timer.run();
}
