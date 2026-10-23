/*
 * Smart Water Quality Analyser (SWQA) - Arduino Nano (ATmega328P)
 * Sensors: TDS (A1), Turbidity (A2), DS18B20 (D2), TCS3200 colour (D3-D7), SSD1306 OLED (I2C)
 *
 * Libraries to install (Arduino IDE > Library Manager):
 *   OneWire, DallasTemperature, Adafruit GFX Library, Adafruit SSD1306
 *
 * Serial monitor (9600 baud) commands:
 *   c        -> calibrate with CLEAR/DISTILLED water (turbidity baseline + colour reference)
 *   t707     -> TDS calibration: probe in a known solution (e.g. 707 ppm), send its value
 *   r        -> reset calibration to defaults
 */

#include <Wire.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pins ----------------
#define TDS_PIN      A1
#define TURB_PIN     A2
#define ONE_WIRE_PIN 2
#define CS_S0 3
#define CS_S1 4
#define CS_S2 5
#define CS_S3 6
#define CS_OUT 7        // TCS3200 OE pin goes to GND (always enabled)

// ---------------- OLED ----------------
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ---------------- Temperature ----------------
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

// ---------------- Calibration (stored in EEPROM) ----------------
struct Calib {
  uint16_t magic;
  float tdsK;          // TDS correction factor
  float turbClearV;    // voltage of clear water in the turbidity sensor
  float refR, refG, refB;  // TCS3200 periods (us) for clear water
};
Calib cal;
const uint16_t MAGIC = 0x5351;

void setDefaults() {
  cal.magic = MAGIC;
  cal.tdsK = 1.0;
  cal.turbClearV = 4.1;
  cal.refR = cal.refG = cal.refB = 200.0;
}
void loadCal() {
  EEPROM.get(0, cal);
  if (cal.magic != MAGIC) { setDefaults(); EEPROM.put(0, cal); }
}
void saveCal() { EEPROM.put(0, cal); }

// ---------------- Sampling ----------------
#define N 15
int tdsBuf[N], turbBuf[N];
byte idx = 0;

unsigned long tSample = 0, tTemp = 0;
bool tempPending = false;

float tempC = 25.0;
float tdsPpm = 0, turbV = 0, turbNTU = 0, clarityPct = 100;
float cr = 1, cg = 1, cb = 1;     // colour ratios vs clear water (1.0 = same as clear)
const char *colorName = "Clear";
const char *verdict = "OK";

int medianOf(int *src) {
  int a[N];
  for (byte i = 0; i < N; i++) a[i] = src[i];
  for (byte i = 0; i < N - 1; i++)
    for (byte j = 0; j < N - 1 - i; j++)
      if (a[j] > a[j + 1]) { int t = a[j]; a[j] = a[j + 1]; a[j + 1] = t; }
  return a[N / 2];
}

// ---------------- TCS3200 ----------------
unsigned long readChannel(bool s2, bool s3) {
  digitalWrite(CS_S2, s2);
  digitalWrite(CS_S3, s3);
  delay(8);                                   // let the filter settle
  return pulseIn(CS_OUT, LOW, 25000UL);       // period in us (lower = more light)
}
void readColor(unsigned long &r, unsigned long &g, unsigned long &b) {
  r = readChannel(LOW, LOW);
  b = readChannel(LOW, HIGH);
  g = readChannel(HIGH, HIGH);
}

// ---------------- Calculations ----------------
void updateReadings() {
  // --- TDS (temperature compensated) ---
  float v = median_tds() * 5.0 / 1024.0;
  float comp = 1.0 + 0.02 * (tempC - 25.0);
  float vc = v / comp;
  float raw = (133.42 * vc * vc * vc - 255.86 * vc * vc + 857.39 * vc) * 0.5;
  tdsPpm = raw * cal.tdsK;
  if (tdsPpm < 0) tdsPpm = 0;

  // --- Turbidity ---
  turbV = median_turb() * 5.0 / 1024.0;
  clarityPct = constrain(turbV / cal.turbClearV * 100.0, 0, 100);
  float vn = turbV * (4.2 / cal.turbClearV);   // normalise to the reference curve
  if (vn >= 4.2) turbNTU = 0;
  else if (vn < 2.5) turbNTU = 3000;
  else turbNTU = -1120.4 * vn * vn + 5742.3 * vn - 4352.9;
  if (turbNTU < 0) turbNTU = 0;

  // --- Colour ---
  unsigned long r, g, b;
  readColor(r, g, b);
  if (r && g && b) {
    cr = constrain(cal.refR / r, 0, 1.2);
    cg = constrain(cal.refG / g, 0, 1.2);
    cb = constrain(cal.refB / b, 0, 1.2);
    float lo = min(cr, min(cg, cb));
    if (lo > 0.85) colorName = "Clear";
    else if (cb <= cr && cb <= cg) colorName = "Yellow/Brown";
    else if (cr <= cg && cr <= cb) colorName = "Greenish";
    else colorName = "Tinted";
  } else {
    colorName = "No signal";
  }

  // --- Simple verdict ---
  if (tdsPpm > 900 || turbNTU > 25) verdict = "UNSAFE";
  else if (tdsPpm > 500 || turbNTU > 5) verdict = "POOR";
  else if (tdsPpm > 300 || turbNTU > 1) verdict = "FAIR";
  else verdict = "GOOD";
}
int median_tds()  { return medianOf(tdsBuf); }
int median_turb() { return medianOf(turbBuf); }

// ---------------- Output ----------------
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Temp : ")); display.print(tempC, 1); display.println(F(" C"));
  display.print(F("TDS  : ")); display.print(tdsPpm, 0); display.println(F(" ppm"));
  display.print(F("Turb : ")); display.print(turbNTU, 0); display.println(F(" NTU"));
  display.print(F("Clear: ")); display.print(clarityPct, 0); display.println(F(" %"));
  display.print(F("Color: ")); display.println(colorName);
  display.setTextSize(2);
  display.setCursor(0, 48);
  display.print(verdict);
  display.display();
}

void printSerial() {
  Serial.print(F("T=")); Serial.print(tempC, 1);
  Serial.print(F("C  TDS=")); Serial.print(tdsPpm, 0);
  Serial.print(F("ppm  Turb=")); Serial.print(turbNTU, 0);
  Serial.print(F("NTU (")); Serial.print(turbV, 2);
  Serial.print(F("V)  Color=")); Serial.print(colorName);
  Serial.print(F("  ")); Serial.println(verdict);
}

// ---------------- Serial commands ----------------
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'c') {
    unsigned long r, g, b;
    readColor(r, g, b);
    cal.turbClearV = median_turb() * 5.0 / 1024.0;
    if (r && g && b) { cal.refR = r; cal.refG = g; cal.refB = b; }
    saveCal();
    Serial.println(F("Clear-water calibration saved."));
  } else if (c == 't') {
    float target = Serial.parseFloat();
    if (target > 0) {
      float v = median_tds() * 5.0 / 1024.0;
      float vc = v / (1.0 + 0.02 * (tempC - 25.0));
      float raw = (133.42 * vc * vc * vc - 255.86 * vc * vc + 857.39 * vc) * 0.5;
      if (raw > 1) { cal.tdsK = target / raw; saveCal(); Serial.println(F("TDS calibration saved.")); }
    }
  } else if (c == 'r') {
    setDefaults(); saveCal();
    Serial.println(F("Calibration reset."));
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(9600);
  loadCal();

  pinMode(CS_S0, OUTPUT); pinMode(CS_S1, OUTPUT);
  pinMode(CS_S2, OUTPUT); pinMode(CS_S3, OUTPUT);
  pinMode(CS_OUT, INPUT);
  digitalWrite(CS_S0, HIGH); digitalWrite(CS_S1, LOW);   // 20% frequency scaling

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);               // non-blocking conversion

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {      // try 0x3D if blank
    Serial.println(F("OLED not found"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("Smart Water Quality"));
  display.println(F("      Analyser"));
  display.display();

  for (byte i = 0; i < N; i++) { tdsBuf[i] = analogRead(TDS_PIN); turbBuf[i] = analogRead(TURB_PIN); }
  delay(1500);
}

void loop() {
  unsigned long now = millis();

  // fast analog sampling every 40 ms
  if (now - tSample >= 40) {
    tSample = now;
    tdsBuf[idx] = analogRead(TDS_PIN);
    turbBuf[idx] = analogRead(TURB_PIN);
    idx = (idx + 1) % N;
  }

  // start a temperature conversion (does not block)
  if (!tempPending && now - tTemp >= 1000) {
    tempSensor.requestTemperatures();
    tTemp = now;
    tempPending = true;
  }

  // read it after ~800 ms, then refresh everything
  if (tempPending && now - tTemp >= 800) {
    float t = tempSensor.getTempCByIndex(0);
    if (t > -100) tempC = t;
    tempPending = false;
    updateReadings();
    updateDisplay();
    printSerial();
  }

  handleSerial();
}
