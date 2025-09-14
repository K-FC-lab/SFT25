#define BLYNK_TEMPLATE_ID "TMPL6NlzgPnyd"
#define BLYNK_TEMPLATE_NAME "JELATECH "
#define BLYNK_AUTH_TOKEN "bM397JHfKjszZGzXXxdJDJTn_JfSu0Vt"

#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "LabKom-1";
char pass[] = "00001111";

BlynkTimer timer;

// Virtual Pin Mesin 1
#define SUHU     V0
#define START1   V1
#define BERAT    V2
#define STATUS1  V3

// ===================== I2C =====================
#define I2C_SDA 21
#define I2C_SCL 22

// ===================== LCD =====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== KEYPAD ==================
#define ROWS 4
#define COLS 4
char keyMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
uint8_t rowPins[ROWS] = {32, 33, 25, 26};
uint8_t colPins[COLS] = {27, 14, 12, 13};
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);

// ===================== DS18B20 =================
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ===================== TCS34725 ================
Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_154MS,   // cukup stabil
  TCS34725_GAIN_4X
);

// ===================== MODEL LR ===============
#include "model_lr_multi_3feat.h"  // pastikan file ini ada

// ===================== STATE GLOBAL ===========
volatile float senSuhu = NAN;

// Input angka
String inputBuffer = "";
bool inputActive   = false;
int  beratMinyak   = 0;
const uint8_t MAX_DIGITS = 6;

// Gate kejernihan (boleh kamu ubah/kalibrasi nanti)
float clear_ref       = 1000.0f;   // referensi C (bisa di-set dari sampel jernih)
float clarity_min_pct = 60.0f;     // ambang minimal % dari clear_ref

// Tampilan LCD
enum UiMode { UI_IDLE, UI_SHOW_WEIGHT, UI_INPUT, UI_SHOW_RESULT, UI_SHOW_MSG };
volatile UiMode uiMode = UI_IDLE;
uint32_t uiModeSinceMs = 0;

// Scan / ML
volatile bool scanRequested = false;
volatile bool scanBusy = false;
String lastResult = "";     // "BISA DIOLAH" / "TIDAK BISA DIOLAH"
uint16_t lastCraw = 0;      // C terakhir dari TCS

// ===================== RTOS ====================
TaskHandle_t thKeypad, thTemp, thScan, thLCD;

// ===================== UTIL ====================
inline uint32_t now_ms() { return millis(); }

void lcdPrintCenter(uint8_t row, const String &s) {
  String t = s; if (t.length() > 16) t.remove(16);
  int pad = max(0, (16 - (int)t.length())/2);
  lcd.setCursor(0,row);
  for (int i=0;i<pad;i++) lcd.print(' ');
  lcd.print(t);
  for (int i=pad+(int)t.length(); i<16; i++) lcd.print(' ');
}

void setUi(UiMode m) {
  uiMode = m;
  uiModeSinceMs = now_ms();
}

// ===================== FEATURE READ ===========
void readColorRaw(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  tcs.getRawData(&r,&g,&b,&c);
}

bool readColorNormalized(float &rN, float &gN, float &bN, uint16_t &cOut) {
  uint16_t r,g,b,c;
  readColorRaw(r,g,b,c);
  if (c == 0) return false;
  rN = (float)r / (float)c;
  gN = (float)g / (float)c;
  bN = (float)b / (float)c;
  cOut = c;
  return true;
}

float readTemperatureC() {
  ds18b20.requestTemperatures();
  return ds18b20.getTempCByIndex(0);
}

// f1,f2,f3 = (rN,gN,bN)*1000; (model 3 fitur). Jika punyamu 4 fitur, tinggal tambahkan suhu sebagai feats[3].
bool readFeatures(float *feats, int avgN = 5) {
  float rSum=0, gSum=0, bSum=0;
  uint32_t cSum=0;
  for (int i=0;i<avgN;i++){
    float rN,gN,bN; uint16_t cNow;
    if (!readColorNormalized(rN,gN,bN,cNow)) return false;
    rSum += rN; gSum += gN; bSum += bN; cSum += cNow;
    vTaskDelay(pdMS_TO_TICKS(12)); // non-blocking jeda antar-sample
  }
  float f1 = (rSum/avgN) * 1000.0f;
  float f2 = (gSum/avgN) * 1000.0f;
  float f3 = (bSum/avgN) * 1000.0f;
  lastCraw = (uint16_t)(cSum/avgN);
  if (LR_NUM_FEATS == 3) {
    feats[0]=f1; feats[1]=f2; feats[2]=f3;
    return true;
  } else {
    // jika modelmu 4 fitur, aktifkan baris berikut:
    // float f4 = readTemperatureC();
    // feats[0]=f1; feats[1]=f2; feats[2]=f3; feats[3]=f4;
    return false; // ganti jadi true jika sudah menambah fitur ke-4
  }
}

// ================= INFERENCE (LR MULTI) =================
static inline void softmax(const float* z, int n, float* p) {
  float m = z[0];
  for (int i=1;i<n;i++) if (z[i]>m) m=z[i];
  float sum=0.f;
  for (int i=0;i<n;i++){ p[i]=expf(z[i]-m); sum+=p[i]; }
  float inv = 1.0f/sum;
  for (int i=0;i<n;i++) p[i]*=inv;
}

int infer_logreg_multi_feats(const float* feats, float* out_prob) {
  float z[LR_NUM_CLASSES];
  for (int c=0;c<LR_NUM_CLASSES;c++){
    float s = LR_B[c];
    for (int i=0;i<LR_NUM_FEATS;i++){
      float xn = (feats[i]-LR_MEAN[i]) / LR_STD[i];
      s += LR_W[c][i]*xn;
    }
    z[c]=s;
  }
  softmax(z, LR_NUM_CLASSES, out_prob);
  int argmax=0; for (int c=1;c<LR_NUM_CLASSES;c++) if (out_prob[c]>out_prob[argmax]) argmax=c;
  return argmax;
}

// ================= KEYPAD LOGIC =================
void processKey(char key) {
  if (key == '*') {
    // Masuk mode input berat
    inputActive = true;
    inputBuffer = "";
    setUi(UI_INPUT);
    Serial.println("[INPUT] Mulai masukkan angka...");
  }
  else if (key == '#') {
    // Selesai input
    if (inputActive && inputBuffer.length() > 0) {
      beratMinyak = inputBuffer.toInt();
      Serial.print("[INPUT] Selesai. Berat = ");
      Serial.println(beratMinyak);
      inputActive = false;
      setUi(UI_SHOW_WEIGHT); // tampilkan berat sebentar
    } else {
      // tidak ada angka
      inputActive = false;
      setUi(UI_IDLE);
      Serial.println("[INPUT] Kosong.");
    }
  }
  else if (inputActive && isDigit(key)) {
    if (inputBuffer.length() < MAX_DIGITS) {
      inputBuffer += key;
      Serial.print("[INPUT] -> "); Serial.println(inputBuffer);
      setUi(UI_INPUT);
    } else {
      Serial.println("[INPUT] Maks 6 digit");
    }
  }
  else if (inputActive && key == 'D') {
    if (inputBuffer.length() > 0) {
      inputBuffer.remove(inputBuffer.length()-1);
      Serial.print("[INPUT] Hapus -> "); Serial.println(inputBuffer);
      setUi(UI_INPUT);
    }
  }
  else if (key == 'A') {
    // MULAI SCAN (hanya jika berat sudah diisi)
    if (beratMinyak <= 0) {
      lastResult = "Set berat dulu!";
      setUi(UI_SHOW_MSG); // tampilkan pesan singkat
    } else if (!scanBusy) {
      scanRequested = true; // minta task scan jalan
    }
  }
  // (opsional) tombol untuk set referensi kejernihan cepat:
  else if (key == 'B') {
    // set clear_ref dari C saat ini (1 sample cepat)
    uint16_t r,g,b,c; readColorRaw(r,g,b,c);
    clear_ref = (float)c;
    Serial.printf("[REF] clear_ref=%.1f\n", clear_ref);
    lastResult = "Ref diset";
    setUi(UI_SHOW_MSG);
  } else if (key == 'C') {
    // naik turunkan ambang (contoh +5%)
    clarity_min_pct += 5.0f;
    if (clarity_min_pct>95.0f) clarity_min_pct=95.0f;
    char buf[20]; snprintf(buf,sizeof(buf),"Th:%.0f%%", clarity_min_pct);
    lastResult = buf; setUi(UI_SHOW_MSG);
  }
}

// ================= TASKS ======================
void taskKeypad(void* pv) {
  for (;;) {
    char key = keypad.getKey();
    if (key) {
      Serial.print("[KEY] "); Serial.println(key);
      processKey(key);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void taskTemp(void* pv) {
  ds18b20.begin();
  for (;;) {
    senSuhu = readTemperatureC();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task untuk menjalankan SCAN + ML saat diminta
void taskScan(void* pv) {
  // cek TCS
  if (!tcs.begin()) {
    Serial.println("[TCS] ERROR: tidak terdeteksi!");
  } else {
    Serial.println("[TCS] OK");
  }
  for (;;) {
    if (scanRequested && !scanBusy) {
      scanRequested = false;
      scanBusy = true;

      // baca fitur
      float feats[LR_NUM_FEATS];
      bool ok = readFeatures(feats, 7);
      if (!ok) {
        lastResult = "Scan gagal";
        setUi(UI_SHOW_MSG);
        scanBusy = false;
        continue;
      }

      // clarity
      float clarity_pct = (clear_ref > 0) ? ((float)lastCraw / clear_ref * 100.0f) : -1.0f;

      // infer ML
      float probs[LR_NUM_CLASSES];
      int cls = infer_logreg_multi_feats(feats, probs);
      float conf = probs[cls];

      Serial.printf("[ML] Class=%s (%.0f%%), C=%u, Clarity=%.1f%%, T=%.1fC, Berat=%d mL\n",
        LR_CLASS_NAMES[cls], conf*100.0f, lastCraw, clarity_pct, senSuhu, beratMinyak);

      // aturan “bisa diolah” berbasis kejernihan
      if (clarity_pct >= 0 && clarity_pct >= clarity_min_pct) {
        lastResult = "LILIN";
      } else {
        lastResult = "OIL SOLIDIFIER";
      }
      setUi(UI_SHOW_RESULT);

      scanBusy = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// LCD: hanya 3 mode tampilan sesuai permintaan
void taskLCD(void* pv) {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  setUi(UI_IDLE);

  for (;;) {
    lcd.clear();
    switch (uiMode) {
      case UI_INPUT: {
        lcd.setCursor(0,0); lcdPrintCenter(0, "Input:");
        String view = inputBuffer; if (view.length()>16) view=view.substring(view.length()-16);
        lcdPrintCenter(1, view);
      } break;

      case UI_SHOW_WEIGHT: {
        char buf[20]; snprintf(buf,sizeof(buf),"Berat: %d mL", beratMinyak);
        lcdPrintCenter(0, buf);
        lcdPrintCenter(1, " ");
        // tampilkan 1.5 detik lalu idle
        if (now_ms() - uiModeSinceMs > 1500) setUi(UI_IDLE);
      } break;

      case UI_SHOW_RESULT: {
        // baris 0: hasil
        lcdPrintCenter(0, lastResult);
        // baris 1: info ringkas (opsional)
        char line[20]; snprintf(line,sizeof(line),"C:%u  T:%.1f", lastCraw, isnan(senSuhu)?0.0:senSuhu);
        lcdPrintCenter(1, String(line));
        // tampilkan 2.5 detik lalu idle
        if (now_ms() - uiModeSinceMs > 2500) setUi(UI_IDLE);
      } break;

      case UI_SHOW_MSG: {
        lcdPrintCenter(0, lastResult);
        lcdPrintCenter(1, " ");
        if (now_ms() - uiModeSinceMs > 1200) setUi(UI_IDLE);
      } break;

      case UI_IDLE:
      default: {
        lcdPrintCenter(0, "JELATECH");
        lcdPrintCenter(1, "SMANEKA");
      } break;
    }

    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ================== BLYNK ==================
void sendToBlynk() {
  Blynk.virtualWrite(SUHU, senSuhu);
  Blynk.virtualWrite(BERAT, beratMinyak);
  Blynk.virtualWrite(STATUS1, lastResult);
  Blynk.virtualWrite(START1, scanRequested);
}

// ================== BLYNK INPUT HANDLER ==================
// Handler untuk menerima input berat dari Blynk (misalnya Numeric Input)
BLYNK_WRITE(BERAT) {
  beratMinyak = param.asInt();   // ambil nilai berat dari Blynk
  Serial.print("[BLYNK] Berat diterima: ");
  Serial.println(beratMinyak);
  setUi(UI_SHOW_WEIGHT);         // tampilkan di LCD sebentar
}

// Handler untuk tombol start dari Blynk (misalnya Button V1)
BLYNK_WRITE(START1) {
  int val = param.asInt();  // 1 = tombol ditekan, 0 = dilepas
  if (val == 1) {
    if (beratMinyak <= 0) {
      lastResult = "Set berat dulu!";
      setUi(UI_SHOW_MSG);
    } else if (!scanBusy) {
      scanRequested = true;
      Serial.println("[BLYNK] Tombol START ditekan, scan dimulai.");
    }
  }
}

void taskBlynk(void* pv) {
  for (;;) {
    Blynk.run();        // jalankan event loop Blynk
    sendToBlynk();      // kirim data ke Blynk
    vTaskDelay(pdMS_TO_TICKS(1000)); // interval 1 detik
  }
}

// ================== SETUP/LOOP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // koneksi WiFi + Blynk

  // Tasks
  xTaskCreatePinnedToCore(taskKeypad, "KEYPAD", 2048, nullptr, 2, &thKeypad, 0);
  xTaskCreatePinnedToCore(taskTemp,   "TEMP",   2048, nullptr, 2, &thTemp,   1);
  xTaskCreatePinnedToCore(taskScan,   "SCAN",   4096, nullptr, 2, &thScan,   1);
  xTaskCreatePinnedToCore(taskLCD,    "LCD",    2048, nullptr, 1, &thLCD,    1);
  xTaskCreatePinnedToCore(taskBlynk, "BLYNK", 4096, nullptr, 2, nullptr, 1);
}

void loop() {
  // kosongkan atau tidur sopan
  vTaskDelay(pdMS_TO_TICKS(1000));
}
