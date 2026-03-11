/*
 * ============================================================
 *  MAGNETIC FIELD PROJECT — ESP32 FIRMWARE  v6
 * ─────────────────────────────────────────────────────────────
 *  BTN1 GPIO4  → Solar wave coil (กดติด ปล่อยดับ + timeout)
 *  BTN2 GPIO5  → Main coil toggle switch (ON/OFF state)
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

// ============================================================
//  CONFIG
// ============================================================
const char*    WIFI_SSID     = "U DONT WANNA KNOW";
const char*    WIFI_PASSWORD = "U DONT WANNA KNOW";
const char*    WS_HOST       = "U DONT WANNA KNOW";
const uint16_t WS_PORT       = 3000;
const char*    WS_PATH       = "/ws";

// ============================================================
//  PINS
// ============================================================
#define SDA_PIN  21
#define SCL_PIN  22
#define CurrentPin 35
#define ENA_PIN  25
#define ENB_PIN  14
#define IN1_PIN  26   // main coil
#define IN2_PIN  27   // main coil
#define IN3_PIN  12   // solar wave coil
#define IN4_PIN  13   // solar wave coil
#define HALL_PIN 34
#define BTN1_PIN  4   // solar wave  — momentary (กดติดปล่อยดับ)
#define BTN2_PIN  5   // main coil   — toggle switch (ON/OFF)
#define PWM_CH    0
#define PWM_CH2   1
#define PWM_FREQ  5000
#define PWM_RES   8
#define STEP_IN1 19
#define STEP_IN2 18
#define STEP_IN3 17
#define STEP_IN4 16
#define HMC5883_ADDR 0x1E
#define OLED_ADDR    0x3C
#define OLED_W 128
#define OLED_H  64

// ============================================================
//  OBJECTS
// ============================================================
Adafruit_SH1106G display(OLED_W, OLED_H, &Wire, -1);
WebSocketsClient  webSocket;

// ============================================================
//  NON-BLOCKING STEPPER
// ============================================================
const uint8_t STEP_SEQ[8][4] = {
  {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},
  {0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}
};
int stepIdx = 0;
uint32_t lastStepUs = 0;
const uint32_t STEP_US = 2441;

void stepperPins(int i) {
  digitalWrite(STEP_IN1, STEP_SEQ[i][0]);
  digitalWrite(STEP_IN2, STEP_SEQ[i][1]);
  digitalWrite(STEP_IN3, STEP_SEQ[i][2]);
  digitalWrite(STEP_IN4, STEP_SEQ[i][3]);
}
void stepperOff() {
  digitalWrite(STEP_IN1, LOW); digitalWrite(STEP_IN2, LOW);
  digitalWrite(STEP_IN3, LOW); digitalWrite(STEP_IN4, LOW);
}
void stepperTick(bool fwd) {
  if (micros() - lastStepUs < STEP_US) return;
  lastStepUs = micros();
  stepIdx = fwd ? (stepIdx + 1) & 7 : (stepIdx - 1 + 8) & 7;
  stepperPins(stepIdx);
}

// ============================================================
//  INTERRUPT FLAGS
// ============================================================
volatile bool btn1Flag = false;   // solar wave — set on FALLING edge

// BTN2 เป็น toggle switch → อ่าน state โดยตรง ไม่ใช้ interrupt
// ใช้ debounce + edge detection ใน loop() แทน

void IRAM_ATTR isr_btn1() { btn1Flag = true; }

// ============================================================
//  STATE
// ============================================================
const float DECL = -0.0158824962f, OFS_X = 15.09f, OFS_Y = 26.14f;
const float SCL_X = 0.981f, SCL_Y = 0.961f;
const float SENS_EFF = 0.92f;

float   hallOffset = 0, heading = 0, gaussValue = 0;
String  compassDir = "N", pole = "None";
uint8_t coilPWM    = 0;
bool    coilForward = true, coilOn = false, wsConnected = false, solarPending = false;

uint32_t lastSend = 0, btn1Last = 0;
const uint32_t SEND_MS = 200, DEBOUNCE_MS = 50;

// BTN2 toggle switch — edge detection
bool     btn2LastState  = HIGH;   // last stable state (INPUT_PULLUP → idle=HIGH)
uint32_t btn2DebounceMs = 0;      // debounce timer

// Solar coil (IN3/IN4) — กดติดปล่อยดับ + auto-off timeout
bool     solarCoilOn  = false;
uint32_t solarCoilEnd = 0;
const uint32_t SOLAR_MS = 10000;

// ============================================================
const float CURR_VREF   = 2592.3f;   // mV — วัดจริงจาก multimeter
const float CURR_SENS   = 66.0f;     // mV/A — ACS712 30A spec
const float CURR_ADC    = 4095.0f;   // 12-bit
float       currOffset  = 2048.0f;   // ADC count ตอนไม่มีกระแส (auto-calibrate)
float       currentAmps = 0.0f;      // ค่ากระแสล่าสุด (A)


// ============================================================
//  OLED STATE
// ============================================================
uint8_t  oledPage = 0;
uint32_t lastPageMs = 0;
const uint32_t PAGE_DUR = 4000;
#define HIST_LEN 20
float    gHist[HIST_LEN] = {};
bool     blinkOn = false;
uint8_t  gHistHead = 0;
uint32_t lastBlink = 0;
uint8_t  dotAnim = 0, spinFrame = 0;

// ============================================================
//  OLED DRAW HELPERS
// ============================================================
void radLine(int cx, int cy, float aDeg, int r0, int r1) {
  float a = (aDeg - 90.0f) * PI / 180.0f;
  display.drawLine(cx + r0*cos(a), cy + r0*sin(a),
                   cx + r1*cos(a), cy + r1*sin(a), SH110X_WHITE);
}

void drawPage0() {
  const int CX = 27, CY = 32, RO = 24, RI = 18;
  display.drawCircle(CX, CY, RO,     SH110X_WHITE);
  display.drawCircle(CX, CY, RO - 1, SH110X_WHITE);
  for (int d = 0; d < 360; d += 45) {
    radLine(CX, CY, (float)d, RO - (d % 90 == 0 ? 6 : 4), RO - 1);
  }
  const char* labels[]  = { "N", "E", "S", "W" };
  const int   degrees[] = {  0,  90, 180, 270  };
  for (int i = 0; i < 4; i++) {
    float a = (degrees[i] - 90) * PI / 180.0f;
    display.setCursor(CX + (RO-9)*cos(a) - 2, CY + (RO-9)*sin(a) - 3);
    display.setTextSize(1); display.print(labels[i]);
  }
  float nr = (heading - 90.0f) * PI / 180.0f;
  int nx = CX + (RI-1)*cos(nr), ny = CY + (RI-1)*sin(nr);
  display.drawLine(CX-1, CY, nx, ny, SH110X_WHITE);
  display.drawLine(CX,   CY, nx, ny, SH110X_WHITE);
  display.drawLine(CX+1, CY, nx, ny, SH110X_WHITE);
  float br = nr + PI;
  display.drawLine(CX, CY, CX+(RI-8)*cos(br), CY+(RI-8)*sin(br), SH110X_WHITE);
  display.fillCircle(CX, CY, 2, SH110X_WHITE);
  display.setTextSize(1); display.setCursor(57, 0);
  display.print(heading, 1); display.print((char)247);
  display.setTextSize(2); display.setCursor(57, 11); display.print(compassDir);
  display.setTextSize(1); display.setCursor(57, 30);
  if (gaussValue >= 0) display.print('+');
  display.print(gaussValue, 1); display.print('G');
  const int GX = 57, GY = 40, GW = 68, GH = 21;
  display.drawRect(GX, GY, GW, GH, SH110X_WHITE);
  float mx = 1.0f;
  for (int i = 0; i < HIST_LEN; i++) if (fabs(gHist[i]) > mx) mx = fabs(gHist[i]);
  int bw = max(1, (GW-2) / HIST_LEN);
  for (int i = 0; i < HIST_LEN; i++) {
    int idx = (gHistHead + i) % HIST_LEN;
    int bh  = (int)(fabs(gHist[idx]) / mx * (GH-2));
    if (bh > 0) display.fillRect(GX+1+i*bw, GY+GH-1-bh, max(bw-1,1), bh, SH110X_WHITE);
  }
}

void drawPage1() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.print("WS:");
  if (wsConnected) {
    display.print("OK ");
    if (blinkOn) display.fillCircle(31, 4, 2, SH110X_WHITE);
    else         display.drawCircle(31, 4, 2, SH110X_WHITE);
  } else { display.print("---"); }
  display.setCursor(66, 0); display.print("POLE:"); display.print(pole);
  display.setCursor(0, 13);
  if (coilOn) {
    display.print("COIL ON ");
    for (int i = 0; i < 4; i++) {
      int dx = 63 + i*14;
      if (i <= (int)dotAnim) display.fillCircle(dx, 16, 3, SH110X_WHITE);
      else                   display.drawCircle(dx, 16, 3, SH110X_WHITE);
    }
  } else { display.print("COIL OFF"); }
  display.setCursor(0, 26); display.print("PWM:");
  int fw = (int)((coilOn ? coilPWM : 0) / 255.0f * 72);
  display.drawRect(26, 26, 74, 8, SH110X_WHITE);
  if (fw > 0) display.fillRect(27, 27, fw, 6, SH110X_WHITE);
  display.setCursor(104, 26); display.print(coilOn ? coilPWM : 0);
  display.setCursor(0, 37); display.print("STEP:");
  if (coilOn) {
    display.print(coilForward ? " CW" : " CCW");
    const int8_t sx[] = { 0, 4, 5, 4, 0,-4,-5,-4 };
    const int8_t sy[] = {-5,-4, 0, 4, 5, 4, 0,-4 };
    for (int s = 0; s < 8; s++) {
      int dx = 97 + sx[s], dy = 41 + sy[s];
      if (s == (int)spinFrame) display.fillCircle(dx, dy, 2, SH110X_WHITE);
      else                     display.drawPixel(dx, dy, SH110X_WHITE);
    }
  } else { display.print(" ---"); }
  display.drawLine(0, 51, OLED_W, 51, SH110X_WHITE);
  display.setCursor(0, 54);
//   display.print(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "No WiFi");
  display.print("Current: "); display.print(currentAmps, 2); display.print('A');
}

void drawBoot(uint8_t f) {
  display.clearDisplay();
  int cx = OLED_W/2, cy = OLED_H/2;
  for (int i = 0; i < 4; i++) {
    int r = (f%8 - i)*5; if (r > 2 && r < 30) display.drawCircle(cx, cy, r, SH110X_WHITE);
  }
  display.fillCircle(cx, cy, 2, SH110X_WHITE);
  if (f >= 6)  { display.setTextSize(1); display.setTextColor(SH110X_WHITE); display.setCursor(cx-24, cy+12); display.print("MAG FIELD"); }
  if (f >= 10) { display.setCursor(cx-18, cy+22); display.print("SYSTEM OK"); }
  display.display();
}

void updateOLED() {
  uint32_t now = millis();
  if (now - lastPageMs > PAGE_DUR) { lastPageMs = now; oledPage = (oledPage + 1) % 2; }
  if (now - lastBlink  > 480)      { lastBlink = now; blinkOn = !blinkOn; dotAnim = (dotAnim+1)%4; spinFrame = (spinFrame+1)%8; }
  gHist[gHistHead] = gaussValue; gHistHead = (gHistHead + 1) % HIST_LEN;
  display.clearDisplay(); display.setTextColor(SH110X_WHITE);
  for (int p = 0; p < 2; p++) {
    if (p == oledPage) display.fillCircle(122 + p*4, 3, 1, SH110X_WHITE);
    else               display.drawCircle(122 + p*4, 3, 1, SH110X_WHITE);
  }
  if (oledPage == 0) drawPage0(); else drawPage1();
  display.display();
}

// ============================================================
//  SENSORS
// ============================================================
void initHMC5883() {
  Wire.beginTransmission(HMC5883_ADDR); Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDR); Wire.write(0x01); Wire.write(0xA0); Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDR); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
  delay(10);
}

void readCompass() {
  Wire.beginTransmission(HMC5883_ADDR); Wire.write(0x03); Wire.endTransmission();
  Wire.requestFrom((uint8_t)HMC5883_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return;
  int16_t x = (Wire.read()<<8)|Wire.read();
  int16_t z = (Wire.read()<<8)|Wire.read();
  int16_t y = (Wire.read()<<8)|Wire.read();
  float xf = (x - OFS_X)*SCL_X, yf = (y - OFS_Y)*SCL_Y;
  float h = atan2(yf, xf) + DECL;
  if (h < 0) h += TWO_PI; if (h > TWO_PI) h -= TWO_PI;
  heading = h * 180.0f / PI;
  if      (heading >= 337 || heading < 22)  compassDir = "N";
  else if (heading < 67)   compassDir = "NE"; else if (heading < 112) compassDir = "E";
  else if (heading < 157)  compassDir = "SE"; else if (heading < 202) compassDir = "S";
  else if (heading < 247)  compassDir = "SW"; else if (heading < 292) compassDir = "W";
  else                     compassDir = "NW";
}

// ============================================================
//  CONFIG — ปรับตรงนี้
// ============================================================
#define HALL_SAMPLES   64       // เพิ่มจาก 4 → 64 ลด noise √16 เท่า
#define HALL_ALPHA     0.15f    // low-pass weight (0.0=sluggish, 1.0=raw)
#define POLE_THRESHOLD 40       // dead zone (เพิ่มจาก 30 ถ้ายัง flip)

float hallSmoothed = 0;         // ค่า EMA ที่ผ่าน filter แล้ว
bool  hallInited   = false;     // init flag สำหรับ EMA

// ============================================================
void readHall() {
  // ── 1. Oversample 64 ครั้ง → ลด random noise ──────────────
  long sum = 0;
  for (int i = 0; i < HALL_SAMPLES; i++) {
    sum += analogRead(HALL_PIN);
    delayMicroseconds(100);     // 64×100µs = 6.4ms รวม
  }
  float adc = sum / (float)HALL_SAMPLES;

  // ── 2. Exponential Moving Average (EMA) ───────────────────
  // กรอง low-frequency drift และ spike ออก
  // หน่วงค่าเดิม (1-α) + ค่าใหม่ (α)
  if (!hallInited) { hallSmoothed = adc; hallInited = true; }  // init ครั้งแรก
  hallSmoothed = hallSmoothed * (1.0f - HALL_ALPHA) + adc * HALL_ALPHA;

  // ── 3. แปลง ADC → Gauss จากค่า smoothed ──────────────────
  gaussValue = ((hallSmoothed - hallOffset) / 4095.0f * 3300.0f) / SENS_EFF;

  // ── 4. Pole detection พร้อม hysteresis ────────────────────
  // hysteresis ป้องกันการ flip กลับไปกลับมารอบ threshold
  float d = hallSmoothed - hallOffset;
  if      (pole != "S" && d >  POLE_THRESHOLD) pole = "S";
  else if (pole != "N" && d < -POLE_THRESHOLD) pole = "N";
  else if (fabs(d) < POLE_THRESHOLD * 0.5f)    pole = "None";  // re-arm zone

  Serial.printf("[HALL] raw=%.1f smooth=%.1f d=%.1f → %.3fG  %s\n",adc, hallSmoothed, d, gaussValue, pole.c_str());
}

void setSolenoid(uint8_t pwm, bool fwd) {
  coilPWM = pwm; coilForward = fwd;
  ledcWrite(PWM_CH, pwm);
  if (pwm == 0) { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, LOW);  }
  else if (fwd) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);  }  //S
  else          { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); } //N
}

void applyCoilState() {
  setSolenoid(coilOn ? coilPWM : 0, coilForward);
  if (!coilOn) stepperOff();
}

// ── Solar coil (IN3/IN4) ──────────────────────────────────
void setSolarCoilHW(bool on) {
  if (on) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);  ledcWrite(PWM_CH2, 200); }
  else    { digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, LOW);  ledcWrite(PWM_CH2, 0);   }
}

void triggerSolarCoil() {
  if (solarCoilOn) {
    solarCoilOn = false;
    setSolarCoilHW(false);
  } else {
    solarCoilOn  = true;
    solarCoilEnd = millis() + SOLAR_MS;
    setSolarCoilHW(true);
  }
}

void tickSolarCoil() {
  if (solarCoilOn && millis() >= solarCoilEnd) {
    solarCoilOn = false;
    setSolarCoilHW(false);
  }
}

// ============================================================
void calibrateCurrent() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Cal Current...");
  display.println("No load!");
  display.display();

  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += analogRead(CurrentPin);
    delayMicroseconds(200);
  }
  currOffset = sum / 500.0f;

  Serial.print("[ACS712] offset ADC = ");
  Serial.print(currOffset, 1);
  Serial.print("  (");
  Serial.print(currOffset / CURR_ADC * CURR_VREF, 1);
  Serial.println(" mV)");
}

// ============================================================
float readCurrentOnce() {
  int   raw  = analogRead(CurrentPin);
  float mv   = (raw   / CURR_ADC) * CURR_VREF;    // raw → mV
  float mvOfs= (currOffset / CURR_ADC) * CURR_VREF;// offset → mV
  return (mv - mvOfs) / CURR_SENS;                 // mV / (mV/A) = A
}
// ============================================================
void readCurrent() {
  const int COUNT = 200;
  double sumSq = 0;

  for (int i = 0; i < COUNT; i++) {
    float c = readCurrentOnce();
    sumSq += c * c;                   // RMS: sum of squares
    delayMicroseconds(200);
  }

  // RMS current — เหมาะกับ PWM load
  currentAmps = sqrt(sumSq / COUNT);

  // ถ้า coil ดับ → ล้างให้เป็น 0 ป้องกัน noise
  if (!coilOn && !solarCoilOn) currentAmps = 0.0f;
}


// ============================================================
//  WEBSOCKET
// ============================================================
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      webSocket.sendTXT("{\"event\":\"esp32_hello\",\"pwm\":" + String(coilPWM) +
                        ",\"dir\":\"" + (coilForward ? "FWD" : "REV") +
                        "\",\"coilOn\":" + (coilOn ? "true" : "false") + "}");
      break;
    case WStype_DISCONNECTED: wsConnected = false; break;
    case WStype_TEXT: {
      String msg = (char*)payload;
      if (msg.indexOf("\"STOP\"") >= 0) {
        coilPWM = 0; setSolenoid(0, coilForward); stepperOff();
      }
      else if (msg.indexOf("\"COIL_TOGGLE\"") >= 0) {
        coilOn = !coilOn; applyCoilState();
        webSocket.sendTXT("{\"event\":\"coilToggle\",\"coilOn\":" + String(coilOn ? "true" : "false") + "}");
      }
      else if (msg.indexOf("\"SET_PWM\"") >= 0) {
        int i = msg.indexOf("\"value\":"); if (i >= 0) { coilPWM = constrain(msg.substring(i+8).toInt(), 0, 255); if (coilOn) setSolenoid(coilPWM, coilForward); }
      }
      else if (msg.indexOf("\"SET_DIR\"") >= 0) {
        coilForward = (msg.indexOf("FWD") >= 0); if (coilOn) setSolenoid(coilPWM, coilForward);
      }
      else if (msg.indexOf("\"SET_COIL\"") >= 0) {
        int p = msg.indexOf("\"pwm\":"); if (p >= 0) coilPWM = constrain(msg.substring(p+6).toInt(), 0, 255);
        coilForward = (msg.indexOf("FWD") >= 0); if (coilOn) setSolenoid(coilPWM, coilForward);
      }
      else if (msg.indexOf("\"SOLAR_WAVE\"") >= 0) {
        triggerSolarCoil();
      }
      break;
    }
    default: break;
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200); delay(300);
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_PIN, PWM_CH);
  ledcWrite(PWM_CH, 0);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB_PIN, PWM_CH2);
  ledcWrite(PWM_CH2, 0);
  pinMode(STEP_IN1, OUTPUT); pinMode(STEP_IN2, OUTPUT);
  pinMode(STEP_IN3, OUTPUT); pinMode(STEP_IN4, OUTPUT);
  stepperOff();

  // BTN1 = momentary → interrupt on FALLING
  pinMode(BTN1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN1_PIN), isr_btn1, FALLING);

  // BTN2 = toggle switch → poll state, no interrupt needed
  pinMode(BTN2_PIN, INPUT_PULLUP);
  btn2LastState = digitalRead(BTN2_PIN);   // read initial switch position

  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
  if (!display.begin(OLED_ADDR, true)) Serial.println("[OLED] FAIL");
  for (uint8_t f = 0; f < 14; f++) { drawBoot(f); delay(100); }
  initHMC5883();
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0); display.println("Calibrating..."); display.println("Remove magnets!"); 
  display.display();

  for (int i = 0; i < 200; i++) { analogRead(HALL_PIN); delay(2); }   // warm-up ทิ้ง
  long s = 0;
  for (int i = 0; i < 1000; i++) { s += analogRead(HALL_PIN); delay(2); }
  hallOffset   = s / 1000.0f;
  hallSmoothed = hallOffset;
  hallInited   = true;
  
  display.clearDisplay(); display.setCursor(0, 0); display.println("WiFi..."); display.display();
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(500);
  Serial.println(WiFi.status() == WL_CONNECTED ? "[WiFi] " + WiFi.localIP().toString() : "[WiFi] FAIL");
  calibrateCurrent();
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(wsEvent);
  webSocket.setReconnectInterval(5000);
  lastPageMs = millis();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  webSocket.loop();
  uint32_t now = millis();

  // ============================================================
  // BTN1 (GPIO4) — Momentary push button → Solar wave coil
  // ISR sets btn1Flag on FALLING edge, debounce here
  // ============================================================
  if (btn1Flag && now - btn1Last > DEBOUNCE_MS) {
    btn1Flag     = false;
    btn1Last     = now;
    solarPending = true;
    triggerSolarCoil();
  }

  // ============================================================
  // BTN2 (GPIO5) — Toggle switch → Main coil ON/OFF
  // No ISR — poll and detect state change (LOW=ON, HIGH=OFF)
  // On state change: sync coilOn with switch position
  // ============================================================
  bool btn2Now = digitalRead(BTN2_PIN);
  if (btn2Now != btn2LastState && now - btn2DebounceMs > DEBOUNCE_MS) {
    btn2DebounceMs = now;
    btn2LastState  = btn2Now;

    // Switch flipped → sync coil state directly (not toggle)
    // LOW  = switch ON  → coilOn = true
    // HIGH = switch OFF → coilOn = false
    coilOn = (btn2Now == LOW);
    applyCoilState();
    webSocket.sendTXT(
      "{\"event\":\"coilToggle\",\"coilOn\":" +
      String(coilOn ? "true" : "false") + "}"
    );
  }

  // ============================================================
  // Stepper + solar coil tick
  // ============================================================
  if (coilOn) stepperTick(coilForward);
  tickSolarCoil();

  // ============================================================
  // Telemetry every SEND_MS
  // ============================================================
  if (now - lastSend >= SEND_MS) {
    lastSend = now;
    readCompass();
    readHall();
    readCurrent();
    updateOLED();

    String j = "{";
    j += "\"heading\":"     + String(heading, 2)                        + ",";
    j += "\"direction\":\"" + compassDir                                + "\",";
    j += "\"gauss\":"       + String(gaussValue, 2)                     + ",";
    j += "\"pole\":\""      + pole                                      + "\",";
    j += "\"pwm\":"         + String(coilPWM)                           + ",";
    j += "\"dir\":\""       + String(coilForward ? "FWD" : "REV")      + "\",";
    j += "\"coilOn\":"      + String(coilOn      ? "true" : "false")   + ",";
    j += "\"solarCoilOn\":" + String(solarCoilOn ? "true" : "false")   + ",";
    j += "\"btn1\":"        + String(digitalRead(BTN1_PIN) == LOW ? "true" : "false") + ",";
    j += "\"btn2\":"        + String(digitalRead(BTN2_PIN) == LOW ? "true" : "false") + ",";
    j += "\"solarWave\":"   + String(solarPending ? "true" : "false");
    j += "}";

    webSocket.sendTXT(j);
    solarPending = false;
  }
}
