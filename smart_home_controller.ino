/*
 * Smart Home Controller
 * Arduino Uno | LCD 16x2 (4-bit parallel) | VL53L0X | Thermistor | LDR | Button | LEDs
 *
 * Pins:
 *   D8  – Mode button (INPUT_PULLUP, active LOW)
 *   D9  – Green LED  (normal mode)
 *   D10  – Red LED    (security / alarm)
 *   D7  – LCD RS
 *   D6  – LCD EN
 *   D5  – LCD D4
 *   D4 – LCD D5
 *   D3 – LCD D6
 *   D2 – LCD D7
 *   A0  – Thermistor (divider: 5V → thermistor → junction → 10kΩ → GND)
 *   A1  – LDR        (divider: 5V → LDR → junction → 10kΩ → GND)
 *   A4  – VL53L0X SDA (I2C)
 *   A5  – VL53L0X SCL (I2C)
 *
 * Libraries required (install via Library Manager):
 *   - LiquidCrystal   (built-in)
 *   - Wire             (built-in)
 *   - VL53L0X by Pololu  (search "VL53L0X" → by Pololu)
 *
 * "Email" alerts print to Serial monitor (9600 baud).
 */

#include <LiquidCrystal.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <stdlib.h>
#include <string.h>

// ── Pin definitions ───────────────────────────────────────────
#define PIN_BUTTON     8
#define PIN_LED_GREEN  9
#define PIN_LED_RED    10
#define PIN_THERMISTOR A0
#define PIN_LDR        A1

// ── LCD: RS, EN, D4, D5, D6, D7 ─────────────────────────────
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// ── VL53L0X ──────────────────────────────────────────────────
VL53L0X rangefinder;

// ── Thermistor constants (Steinhart-Hart / Beta method) ───────
#define THERM_NOMINAL_R   10000   // resistance at 25°C (Ω)
#define THERM_NOMINAL_T   25      // temperature at nominal resistance (°C)
#define THERM_BETA        3950    // Beta coefficient — check your datasheet
#define THERM_SERIES_R    10000   // series resistor value (Ω)

// ── Thresholds ────────────────────────────────────────────────
#define DISTANCE_THRESHOLD_CM  40   // motion detected within this many cm
#define LDR_CHANGE_THRESHOLD   150  // raw ADC change to count as a light event
#define TEMP_HIGH_WARN         35.0 // °C, warn on LCD if exceeded
#define TEMP_LOW_WARN          10.0 // °C, warn on LCD if below

// ── Debounce / timing ─────────────────────────────────────────
#define DEBOUNCE_MS         50
#define LCD_REFRESH_MS    1000   // update LCD every second
#define SENSOR_POLL_MS     250   // poll sensors in security mode
#define ALERT_COOLDOWN_MS 5000  // minimum ms between alerts for same event type
#define DASHBOARD_UPDATE_MS 1000 // send dashboard data over USB serial

// ── Operating modes ───────────────────────────────────────────
enum Mode { NORMAL, SECURITY };
Mode currentMode = NORMAL;

// ── State variables ───────────────────────────────────────────
bool     lastButtonState      = HIGH;   // INPUT_PULLUP: pressed = LOW
bool     buttonState          = HIGH;
unsigned long lastDebounceTime = 0;

unsigned long lastLcdUpdate    = 0;
unsigned long lastSensorPoll   = 0;
unsigned long lastDashboardUpdate = 0;

int      lastLdrValue          = -1;    // initialised on first read
unsigned long lastMotionAlert  = 0;
unsigned long lastLightAlert   = 0;

// ── Helpers ──────────────────────────────────────────────────

/*
 * Read thermistor and return temperature in Celsius using Beta equation.
 * Voltage divider: 5V → thermistor → junction(A0) → 10kΩ → GND
 */
float readTemperature() {
  // Take multiple samples and average to reduce noise
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(PIN_THERMISTOR);
    delay(5);
  }
  float raw = sum / 10.0;

  // Use 4.7V if powered by USB, or 5.0V if powered by barrel jack
  // Adjust this value until room temp reads correctly
  float vRef    = 3.5;
  float voltage = raw * (vRef / 1023.0);
  float tempC   = (voltage - 0.5) * 100.0;
  return tempC;
}

/*
 * Read LDR raw ADC value (0–1023).
 * Higher value = brighter (more voltage across series resistor, less across LDR).
 */
int readLDR() {
  return analogRead(PIN_LDR);
}

/*
 * Read VL53L0X distance in cm. Returns -1 on timeout/out-of-range.
 */
int readDistanceCm() {
  uint16_t mm = rangefinder.readRangeSingleMillimeters();
  if (rangefinder.timeoutOccurred()) return -1;
  return (int)(mm / 10);
}

int buildMonthNumber(const char* buildDate) {
  const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  for (int i = 0; i < 12; i++) {
    if (strncmp(buildDate, months + (i * 3), 3) == 0) {
      return i + 1;
    }
  }
  return 1;
}

bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int daysInMonth(int month, int year) {
  switch (month) {
    case 2:
      return isLeapYear(year) ? 29 : 28;
    case 4:
    case 6:
    case 9:
    case 11:
      return 30;
    default:
      return 31;
  }
}

void formatCurrentDateTime(char* out, size_t len) {
  const char* buildDate = __DATE__; // "Mmm dd yyyy"
  const char* buildTime = __TIME__; // "hh:mm:ss"

  int month = buildMonthNumber(buildDate);
  int day   = atoi(buildDate + 4);
  int year  = atoi(buildDate + 7);
  int hour  = (buildTime[0] - '0') * 10 + (buildTime[1] - '0');
  int min   = (buildTime[3] - '0') * 10 + (buildTime[4] - '0');
  int sec   = (buildTime[6] - '0') * 10 + (buildTime[7] - '0');

  unsigned long totalSeconds = (unsigned long)sec + (millis() / 1000UL);
  sec = totalSeconds % 60;
  min += totalSeconds / 60;
  hour += min / 60;
  min %= 60;
  day += hour / 24;
  hour %= 24;

  while (day > daysInMonth(month, year)) {
    day -= daysInMonth(month, year);
    month++;
    if (month > 12) {
      month = 1;
      year++;
    }
  }

  snprintf(out, len, "%02d/%02d %02d:%02d:%02d", month, day, hour, min, sec);
}

void formatEmailDateTime(char* out, size_t len) {
  const char* buildDate = __DATE__; // "Mmm dd yyyy"
  const char* buildTime = __TIME__; // "hh:mm:ss"

  int month = buildMonthNumber(buildDate);
  int day   = atoi(buildDate + 4);
  int year  = atoi(buildDate + 7);
  int hour  = (buildTime[0] - '0') * 10 + (buildTime[1] - '0');
  int min   = (buildTime[3] - '0') * 10 + (buildTime[4] - '0');
  int sec   = (buildTime[6] - '0') * 10 + (buildTime[7] - '0');

  unsigned long totalSeconds = (unsigned long)sec + (millis() / 1000UL);
  sec = totalSeconds % 60;
  min += totalSeconds / 60;
  hour += min / 60;
  min %= 60;
  day += hour / 24;
  hour %= 24;

  while (day > daysInMonth(month, year)) {
    day -= daysInMonth(month, year);
    month++;
    if (month > 12) {
      month = 1;
      year++;
    }
  }

  const char* meridiem = hour >= 12 ? "PM" : "AM";
  int hour12 = hour % 12;
  if (hour12 == 0) {
    hour12 = 12;
  }

  snprintf(out, len, "%04d-%02d-%02d %d:%02d %s", year, month, day, hour12, min, meridiem);
}

/*
 * Print a formatted "email" alert to the Serial monitor.
 * This replaces internet notification per project modification.
 */
void sendSerialAlert(const char* eventType, int distanceCm, int lightValue) {
  char timeBuf[24];
  formatEmailDateTime(timeBuf, sizeof(timeBuf));

  Serial.println(F("Subject: Arduino Uno Security Alert"));
  Serial.println(F("Security Alert!"));
  Serial.println(F("Mode: Security Mode"));
  Serial.print(F("Event: "));
  Serial.println(eventType);
  Serial.print(F("Ultrasonic distance: "));
  Serial.print(distanceCm < 0 ? 0 : distanceCm);
  Serial.println(F(" cm"));
  Serial.print(F("Light sensor value: "));
  Serial.println(lightValue);
  Serial.print(F("Time: "));
  Serial.println(timeBuf);
}

void sendDashboardData() {
  float temp = readTemperature();
  int ldrVal = readLDR();
  int distCm = readDistanceCm();

  Serial.print(F("DATA,mode="));
  Serial.print(currentMode == SECURITY ? F("Security") : F("Normal"));
  Serial.print(F(",temp="));
  Serial.print(temp, 1);
  Serial.print(F(",light="));
  Serial.print(ldrVal);
  Serial.print(F(",distance="));
  Serial.print(distCm < 0 ? 0 : distCm);
  Serial.print(F(",millis="));
  Serial.println(millis());
}

// ── Mode helpers ─────────────────────────────────────────────

void enterNormalMode() {
  currentMode = NORMAL;
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, LOW);
  lcd.clear();
  lastLdrValue = -1; // reset baseline on mode change
  Serial.println(F("[MODE] Switched to NORMAL mode"));
}

void enterSecurityMode() {
  currentMode = SECURITY;
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, HIGH); // red stays on while security mode is active
  lcd.clear();
  lastLdrValue = readLDR();       // capture baseline immediately
  lastLightAlert = millis() - ALERT_COOLDOWN_MS - 1UL;
  Serial.println(F("[MODE] Switched to SECURITY mode"));
}

// ── Normal mode display ──────────────────────────────────────

void updateNormalDisplay() {
  float temp = readTemperature();
  char dateTimeBuf[17];
  formatCurrentDateTime(dateTimeBuf, sizeof(dateTimeBuf));

  lcd.clear();

  // Row 0: temperature + mode tag
  lcd.setCursor(0, 0);
  lcd.print("Tmp:");
  lcd.print(temp, 1);
  lcd.print((char)223); // degree symbol
  lcd.print("C  HOME");

  // Row 1: current date/time
  lcd.setCursor(0, 1);
  lcd.print(dateTimeBuf);
  lcd.print("  ");

  // Debug to Serial
  Serial.print(F("[NORMAL] Temp="));
  Serial.print(temp, 1);
  Serial.print(F("C  DateTime="));
  Serial.println(dateTimeBuf);
}

// ── Security mode polling ─────────────────────────────────────

void pollSecuritySensors() {
  int ldrVal  = readLDR();
  int distCm  = readDistanceCm();
  unsigned long now = millis();
  bool eventDetected = false;
  char eventBuf[32];

  // ── Event 1: significant light change ──
  if (lastLdrValue >= 0) {
    int ldrDelta = ldrVal - lastLdrValue;
    if (abs(ldrDelta) > LDR_CHANGE_THRESHOLD && (now - lastLightAlert) > ALERT_COOLDOWN_MS) {
      lastLightAlert = now;
      eventDetected  = true;
      strcpy(eventBuf, ldrDelta > 0 ? "Light turned on" : "Light level dropped");

      Serial.print(F("[SECURITY] Light change detected! Direction="));
      Serial.print(ldrDelta > 0 ? F("up") : F("down"));
      Serial.print(F(" Delta="));
      Serial.println(ldrDelta);
    }
  }
  lastLdrValue = ldrVal;

  // ── Event 2: object / person too close ──
  if (!eventDetected && distCm > 0 && distCm < DISTANCE_THRESHOLD_CM) {
    if ((now - lastMotionAlert) > ALERT_COOLDOWN_MS) {
      lastMotionAlert = now;
      eventDetected   = true;
      strcpy(eventBuf,    "Motion detected");

      Serial.print(F("[SECURITY] Motion! Distance="));
      Serial.print(distCm);
      Serial.println(F("cm"));
    }
  }

  // ── Update LCD ──
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SECURITY MODE   ");
  lcd.setCursor(0, 1);
  if (eventDetected) {
    lcd.print("!ALERT!         ");
  } else {
    lcd.print("Dst:");
    lcd.print(distCm < 0 ? 0 : distCm);
    lcd.print("cm L:");
    lcd.print(ldrVal);
  }

  // ── Blink red LED on event ──
  if (eventDetected) {
    for (int i = 0; i < 6; i++) {
      digitalWrite(PIN_LED_RED, HIGH);
      delay(120);
      digitalWrite(PIN_LED_RED, LOW);
      delay(120);
    }
    digitalWrite(PIN_LED_RED, HIGH); // return to steady security indicator
    sendSerialAlert(eventBuf, distCm, ldrVal);
  }
}

// ── Button debounce ──────────────────────────────────────────

/*
 * Read the mode button with debouncing.
 * Returns true on a confirmed press (HIGH→LOW transition).
 */
bool buttonPressed() {
  bool reading = digitalRead(PIN_BUTTON);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  lastButtonState = reading;

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) { // active LOW (INPUT_PULLUP)
        return true;
      }
    }
  }
  return false;
}

// ── Setup ─────────────────────────────────────────────────────

void setup() {
  Serial.begin(9600);
  Serial.println(F("Smart Home Controller starting..."));

  // GPIO
  pinMode(PIN_BUTTON,    INPUT_PULLUP);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED,   LOW);

  // LCD
  lcd.begin(16, 2);
  lcd.print("Smart Home v1.0 ");
  lcd.setCursor(0, 1);
  lcd.print("Initialising... ");
  delay(1500);

  // VL53L0X
  Wire.begin();
  if (!rangefinder.init()) {
    Serial.println(F("ERROR: VL53L0X not found! Check wiring and I2C address."));
    lcd.clear();
    lcd.print("VL53L0X ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring");
    // Halt with blinking red LED so it's obvious
    while (true) {
      digitalWrite(PIN_LED_RED, HIGH); delay(300);
      digitalWrite(PIN_LED_RED, LOW);  delay(300);
    }
  }
  rangefinder.setTimeout(500);
  Serial.println(F("VL53L0X initialised OK"));

  // Start in normal mode
  enterNormalMode();
  Serial.println(F("Ready. Press button to toggle mode."));
}

// ── Loop ──────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // ── Check for mode switch ──
  if (buttonPressed()) {
    if (currentMode == NORMAL) {
      enterSecurityMode();
    } else {
      enterNormalMode();
    }
    lastLcdUpdate   = 0; // force immediate LCD refresh
    lastSensorPoll  = 0;
    lastDashboardUpdate = 0;
  }

  // ── Normal mode: refresh LCD on interval ──
  if (currentMode == NORMAL) {
    if (now - lastLcdUpdate >= LCD_REFRESH_MS) {
      lastLcdUpdate = now;
      updateNormalDisplay();
    }
    // Green LED steady on
    digitalWrite(PIN_LED_GREEN, HIGH);
  }

  // ── Security mode: poll sensors on interval ──
  if (currentMode == SECURITY) {
    if (now - lastSensorPoll >= SENSOR_POLL_MS) {
      lastSensorPoll = now;
      pollSecuritySensors();
    }
  }

  // ── Dashboard: send one parseable USB serial line every second ──
  if (now - lastDashboardUpdate >= DASHBOARD_UPDATE_MS) {
    lastDashboardUpdate = now;
    sendDashboardData();
  }
}
