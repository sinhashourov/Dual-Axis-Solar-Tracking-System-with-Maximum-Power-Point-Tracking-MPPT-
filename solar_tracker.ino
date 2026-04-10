
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ==== CONFIG ====
#define LCD_ADDR   0x27
#define LCD_COLS   16
#define LCD_ROWS   2

const unsigned long LCD_UPDATE_MS = 500;
const unsigned long SERVO_MOVE_MS = 30;
const float DISPLAY_MAX_VOLTAGE = 7.0;
const float LOW_V_THRESHOLD = 1;
const int AVG_N = 6;
// =================

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Servo servoUpDown;

Servo servoLeftRight;

int posUpDown    = 90;
int posLeftRight = 90;
int targetUpDown = 90;
int targetLeftRight = 90;

// LDR pins


const int LDR_Up_Pin    = A2;
const int LDR_Down_Pin  = 2;
const int LDR_Left_Pin  = A1;
const int LDR_Right_Pin = A3;
const int solarPin      = A0;
const bool LDR_Up_Dig   = false;
const bool LDR_Down_Dig = true;

// Temperature & fan pins
const int tempSensorPin = 3;   // LOW = hot -> fan on

const int fanPin        = 12;  // HIGH = fan ON

// Button wiring
const int buttonVccPin  = 8;   // provide VCC to button
const int buttonReadPin = 7;   // read button (external 10k pulldown)
const int buttonLedPin  = 6;   // D6 - toggled by button (default OFF)

// Day/Night LED and Solar indicator
const int solarLedPin    = 13; // solar indicator LED (ON when solar ADC > 200)
const int dayNightLedPin = 4;  // D4 - day/night LED: ON when solar LED is OFF (night)

// buffers & state
int LDR_Up_Value = 0;
int LDR_Down_Value = 0;
int LDR_Left_Value = 0;

int LDR_Right_Value = 0;
int SolarValue = 0;
int solarBuf[AVG_N];
int solarBufIdx = 0;
bool solarBufFilled = false;

// button debounce & toggle state
bool buttonLedState = false;          // current state of D6 LED (toggled)
int lastButtonReading = LOW;
int buttonStableState = LOW;
unsigned long lastDebounceMillis = 0;
const unsigned long debounceDelay = 50;

// timers
unsigned long prevLcdMillis = 0;
unsigned long prevServoMoveMillis = 0;

void setup() {
  // LDRs & ADC
  pinMode(LDR_Up_Pin, INPUT);

  pinMode(LDR_Down_Pin, INPUT);
  pinMode(LDR_Left_Pin, INPUT);
  pinMode(LDR_Right_Pin, INPUT);
  pinMode(solarPin, INPUT);

  // Temp & fan
  pinMode(tempSensorPin, INPUT);
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  // Button wiring
  pinMode(buttonVccPin, OUTPUT);
  digitalWrite(buttonVccPin, HIGH); // supply VCC to push button
  pinMode(buttonReadPin, INPUT);     // external pulldown expected

  // Button LED (D6) default OFF
  pinMode(buttonLedPin, OUTPUT);
  buttonLedState = true;
  digitalWrite(buttonLedPin, LOW);


  // Solar indicator and day/night LED
  pinMode(solarLedPin, OUTPUT);
  digitalWrite(solarLedPin, LOW);
  pinMode(dayNightLedPin, OUTPUT);
  digitalWrite(dayNightLedPin, LOW);

  // Servos
  servoUpDown.attach(9);
  servoLeftRight.attach(11);
  servoUpDown.write(posUpDown);
  servoLeftRight.write(posLeftRight);

  // LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // init solar buffer
  for (int i = 0; i < AVG_N; ++i) solarBuf[i] = 0;

}

int readLDR(int pin, bool dig) {
  if (dig) return digitalRead(pin) ? 1023 : 0;
  return analogRead(pin);
}

void updateSensorsAndTargets() {
  LDR_Up_Value    = readLDR(LDR_Up_Pin, LDR_Up_Dig);
  LDR_Down_Value  = readLDR(LDR_Down_Pin, LDR_Down_Dig);
  LDR_Left_Value  = analogRead(LDR_Left_Pin);
  LDR_Right_Value = analogRead(LDR_Right_Pin);

  int rawSolar = analogRead(solarPin);
  solarBuf[solarBufIdx++] = rawSolar;
  if (solarBufIdx >= AVG_N) { solarBufIdx = 0; solarBufFilled = true; }


  long sum = 0;
  int ct = solarBufFilled ? AVG_N : solarBufIdx;
  for (int i = 0; i < ct; ++i) sum += solarBuf[i];
  SolarValue = (ct > 0) ? (sum / ct) : rawSolar;

  // Solar LED logic: ON when SolarValue > 200 (charging/day)
  if (SolarValue > 200) {
    digitalWrite(solarLedPin, HIGH);
    digitalWrite(dayNightLedPin, LOW); // day -> day/night LED off
  } else {
    digitalWrite(solarLedPin, LOW);
    digitalWrite(dayNightLedPin, HIGH); // night -> day/night LED on
  }

  // Tracker control -> set targets only
  int diffUD = LDR_Up_Value - 

LDR_Down_Value;
  int diffLR = LDR_Left_Value - LDR_Right_Value;
  const int deadzone = 100;

  if (abs(diffUD) > abs(diffLR)) {
    if (abs(diffUD) > deadzone) {
      int shift = constrain(diffUD / 600, -2, 2);
      if (shift != 0) targetUpDown += shift;
      else {
        if (diffUD > 0) targetUpDown += 1;
        else targetUpDown -= 1;
      }
      targetUpDown = constrain(targetUpDown, 10, 170);
    }
  } else {
    if (abs(diffLR) > deadzone) {
      int shift = constrain(diffLR / 600, -2, 2);
      if (shift != 0) targetLeftRight -= shift;
      else {

        if (diffLR > 0) targetLeftRight -= 1;
        else targetLeftRight += 1;
      }
      targetLeftRight = constrain(targetLeftRight, 30, 160);
    }
  }
}

void servoStepper() {
  unsigned long now = millis();
  if (now - prevServoMoveMillis < SERVO_MOVE_MS) return;
  prevServoMoveMillis = now;

  bool moved = false;
  if (posUpDown < targetUpDown) { posUpDown++; moved = true; }
  else if (posUpDown > targetUpDown) { posUpDown--; moved = true; }

  if (posLeftRight < targetLeftRight) { posLeftRight++; moved = true; }
  else if (posLeftRight > targetLeftRight) { posLeftRight--; moved = true; }

  if (moved) {
    servoUpDown.write(posUpDown);
    servoLeftRight.write(posLeftRight);
  }
}

void tempAndFanControl() {
  // Reversed logic: LOW => temp detected -> fan ON
  if (digitalRead(tempSensorPin) == LOW) digitalWrite(fanPin, HIGH);
  else digitalWrite(fanPin, LOW);
}

void handleButton() {
  int reading = digitalRead(buttonReadPin);


  if (reading != lastButtonReading) {
    lastDebounceMillis = millis();
    lastButtonReading = reading;
  }

  if ((millis() - lastDebounceMillis) > debounceDelay) {
    if (reading != buttonStableState) {
      buttonStableState = reading;
      // toggle on rising edge (button pressed -> reading HIGH)
      if (buttonStableState == HIGH) {
        buttonLedState = !buttonLedState;
        digitalWrite(buttonLedPin, buttonLedState ? HIGH : LOW);
      }
    }
  }
}

// Update LCD: Row0 shows voltage, Row1 shows temp status only
void updateLCD() {
  float voltage = ((float)SolarValue * DISPLAY_MAX_VOLTAGE) / 1023.0;
  if (voltage < LOW_V_THRESHOLD) voltage = 0.0;
  if (voltage > DISPLAY_MAX_VOLTAGE) voltage = DISPLAY_MAX_VOLTAGE;

  char vbuf[8];
  dtostrf(voltage, 4, 2, vbuf);

  // Row0: "Voltage: x.xxV"
  char row0[17];
  snprintf(row0, sizeof(row0), "Voltage: %sV", vbuf);

  // Row1: "Temp: HOT" or "Temp: OK"
  bool hot = (digitalRead(tempSensorPin) == LOW);

  char row1[17];
  if (hot) snprintf(row1, sizeof(row1), "Temp: HOT");
  else snprintf(row1, sizeof(row1), "Temp: OK");

  // print rows
  lcd.setCursor(0, 0);
  for (int i = 0; i < LCD_COLS; ++i) lcd.print(' ');
  lcd.setCursor(0, 0);
  lcd.print(row0);

  lcd.setCursor(0, 1);
  for (int i = 0; i < LCD_COLS; ++i) lcd.print(' ');
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void loop() {

  updateSensorsAndTargets();
  servoStepper();
  tempAndFanControl();
  handleButton();

  unsigned long now = millis();
  if (now - prevLcdMillis >= LCD_UPDATE_MS) {
    prevLcdMillis = now;
    updateLCD();
  }
}
