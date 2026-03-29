#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
constexpr int BUTTON   = 14;
constexpr int BUZZER   = 13;
constexpr int P1       = 9;   // Streamer melt wire
constexpr int P2       = 10;  // Not connected
constexpr int P3       = 11;  // Ascent motor ignition
constexpr int P4       = 12;  // Parachute melt wire
constexpr int RLED     = 6;
constexpr int GLED     = 7;
constexpr int BLED     = 8;
constexpr int SD_CS    = BUILTIN_SDCARD;

// ── Tuning ───────────────────────────────────────────────────────────────────
constexpr float XTUNE  = 0, YTUNE = 0;   // Servo neutral trim (degrees)

// ── PD Controller & TVC ──────────────────────────────────────────────────────
constexpr float P_GAIN      = 0.3;
constexpr float D_GAIN      = 0.2;
constexpr float SERVO_X_MULT = 4;
constexpr float SERVO_Y_MULT = 4;
constexpr float MAX_TILT     = 5;        // degrees, TVC deflection limit

// ── Motor & Flight Constants ──────────────────────────────────────────────────
constexpr float BURN_TIME      = 3.45;   // s
constexpr float AV_THRUST      = 14.34;  // N
constexpr float ROCKET_WEIGHT  = 0.78;    // kg
constexpr float G              = 9.81;   // m/s²
constexpr float IGN_DELAY      = 0.2;    // s, motor ignition lag
constexpr float SEA_LEVEL_HPA  = 1013.25;

// ── Sensor Smoothing ─────────────────────────────────────────────────────────
constexpr float GYRO_ALPHA    = 0.9;
constexpr float ANGVEL_ALPHA  = 0.9;
constexpr float VEL_ALPHA     = 0.2;     // weight on new velocity sample

// ── Hardware ─────────────────────────────────────────────────────────────────
PWMServo        servoX, servoY;
MPU6050         mpu6050(Wire);
Adafruit_BMP280 bmp;
File            logFile;
char            logFilename[20];

// ── Flight State ─────────────────────────────────────────────────────────────
float altitude, initial_alt, highest_alt, vert_vel;
float gyro_x, gyro_y, gyro_z;
float ang_vel_x, ang_vel_y;
float accel_x, accel_y, accel_z;
float tiltX, tiltY;
bool  poweredFlight = false;  // true only during motor burn — gates emergency deploy

// ── Pyro System ──────────────────────────────────────────────────────────────
struct PyroChannel { int pin; bool active; unsigned long startTime; };

PyroChannel pyros[] = {
  {P1, false, 0},
  {P2, false, 0},
  {P3, false, 0},
  {P4, false, 0}
};

void triggerPyro(int pin) {
  for (auto& ch : pyros) {
    if (ch.pin == pin && !ch.active) {
      digitalWrite(pin, HIGH);
      ch.active    = true;
      ch.startTime = millis();
    }
  }
}

void updatePyros() {
  for (auto& ch : pyros) {
    if (ch.active && millis() - ch.startTime >= 1000) {
      digitalWrite(ch.pin, LOW);
      ch.active = false;
    }
  }
}

// ── Utilities ────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r);
  digitalWrite(GLED, !g);
  digitalWrite(BLED, !b);
}

void beep(int freq, int dur) {
  tone(BUZZER, freq);
  delay(dur);
  noTone(BUZZER);
}

// ── SD Logging ───────────────────────────────────────────────────────────────
void createUniqueLogFile() {
  int idx = 0;
  do { sprintf(logFilename, "LOG%03d.CSV", idx++); }
  while (SD.exists(logFilename) && idx < 1000);

  logFile = SD.open(logFilename, FILE_WRITE);
  if (logFile) {
    Serial.print(F("Logging to: ")); Serial.println(logFilename);
    logFile.println(F("Time(ms),Altitude(m),VertVel(m/s),"
                      "GyroX,GyroY,GyroZ,AngVelX,AngVelY,"
                      "AccelX,AccelY,AccelZ,ServoX,ServoY"));
    logFile.close();
  } else {
    Serial.println(F("Failed to create log file!"));
  }
}

void logData() {
  logFile = SD.open(logFilename, FILE_WRITE);
  if (!logFile) return;
  char buf[128];
  snprintf(buf, sizeof(buf),
    "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    millis(), altitude, vert_vel,
    gyro_x, gyro_y, gyro_z,
    ang_vel_x, ang_vel_y,
    accel_x, accel_y, accel_z,
    constrain(tiltX, -MAX_TILT, MAX_TILT),
    constrain(tiltY, -MAX_TILT, MAX_TILT));
  logFile.println(buf);
  logFile.close();
}

// ── Sensors ───────────────────────────────────────────────────────────────────
void sensors() {
  static unsigned long prevTime = millis();
  static float prev_alt = 0;

  mpu6050.update();

  // X axis negated to match rocket mounting orientation
  float raw_gx  = -(mpu6050.getAngleX() + XTUNE);
  float raw_gy  =   mpu6050.getAngleY() + YTUNE;
  float raw_gz  =   mpu6050.getAngleZ();
  float raw_avx = -mpu6050.getGyroX();
  float raw_avy =  mpu6050.getGyroY();

  gyro_x   = GYRO_ALPHA   * raw_gx  + (1 - GYRO_ALPHA)   * gyro_x;
  gyro_y   = GYRO_ALPHA   * raw_gy  + (1 - GYRO_ALPHA)   * gyro_y;
  gyro_z   = GYRO_ALPHA   * raw_gz  + (1 - GYRO_ALPHA)   * gyro_z;
  ang_vel_x = ANGVEL_ALPHA * raw_avx + (1 - ANGVEL_ALPHA) * ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA * raw_avy + (1 - ANGVEL_ALPHA) * ang_vel_y;

  accel_x = mpu6050.getAccX() * G;
  accel_y = mpu6050.getAccY() * G;
  accel_z = mpu6050.getAccZ() * G;

  altitude = bmp.readAltitude(SEA_LEVEL_HPA) - initial_alt;
  if (altitude > highest_alt) highest_alt = altitude;

  unsigned long now     = millis();
  unsigned long elapsed = now - prevTime;
  if (elapsed >= 50) {
    float raw_vel = (altitude - prev_alt) / (elapsed / 1000.0f);
    vert_vel  = VEL_ALPHA * raw_vel + (1 - VEL_ALPHA) * vert_vel;
    prev_alt  = altitude;
    prevTime  = now;
    logData();
  }

  updatePyros();
  emergency();
}

// ── Emergency ─────────────────────────────────────────────────────────────────
void emergency() {
  if (!poweredFlight) return;                          // only active during motor burn
  if (abs(gyro_x) <= 90 && abs(gyro_y) <= 90) return;

  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
  Serial.println(F("EMERGENCY"));
  LED(true, false, false);
  triggerPyro(P4);   // parachute first in emergency
  delay(1000);
  triggerPyro(P1);   // then streamer
  while (true) {
    updatePyros();
    beep(500, 50);
    delay(50);
    if (digitalRead(BUTTON) == HIGH) while (true) updatePyros();
  }
}

// ── Arm Sequence ─────────────────────────────────────────────────────────────
bool buttonCount() {
  constexpr int  PRESS_WINDOW  = 300;
  constexpr int  PRESSES_REQD  = 5;
  static unsigned long lastPress = 0;
  static int pressCount = 0;

  if (digitalRead(BUTTON) == HIGH) {
    if (millis() - lastPress <= PRESS_WINDOW) {
      pressCount++;
      LED(false, true, false);
      tone(BUZZER, 311.13f * pow(2.0f, pressCount / 12.0f));
      while (digitalRead(BUTTON) == HIGH) {}
      noTone(BUZZER);
      LED(false, false, false);
    } else {
      pressCount = 1;
    }
    lastPress = millis();
  }
  return pressCount > PRESSES_REQD;
}

// ── Countdown & Launch ────────────────────────────────────────────────────────
void countdown() {
  constexpr int DURATION = 30;

  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
  createUniqueLogFile();

  for (int i = DURATION; i > 0; i--) {
    Serial.println(i);
    if (i > 5) {
      // Slow beep phase
      LED(true, false, false);
      beep(440, 200);
      LED(false, false, false);
      delay(800);
    } else if (i > 3) {
      // Solid tone phase
      LED(true, false, false);
      tone(BUZZER, 440);
      delay(1000);
    } else if (i == 3) {
      // Purple + high tone = calibrating (rocket must be still and upright)
      LED(true, false, true);
      tone(BUZZER, 880);
      mpu6050.calcGyroOffsets(true, 0, 0);
      initial_alt = bmp.readAltitude(SEA_LEVEL_HPA);
      Serial.print(F("  baseline alt: ")); Serial.println(initial_alt);
      noTone(BUZZER);
      LED(false, false, false);
    }
    // i=2, i=1 fall through — calibration already consumed the time
  }

  Serial.println(F("LAUNCH"));
  triggerPyro(P3);
}

// ── TVC ───────────────────────────────────────────────────────────────────────
void TVC() {
  sensors();
  tiltX = constrain(P_GAIN * gyro_x + D_GAIN * ang_vel_x, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  tiltY = constrain(P_GAIN * gyro_y + D_GAIN * ang_vel_y, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
  servoX.write(tiltX + 90 + XTUNE);
  servoY.write(tiltY + 90 + YTUNE);
}

// ── Descent (disabled — landing legs not yet tested) ─────────────────────────
void descent() {
  constexpr float DELAY_DROP = 0.5f * G * IGN_DELAY * IGN_DELAY;
  constexpr float DELTA_ALT  = 0.5f * ((AV_THRUST / ROCKET_WEIGHT) - G)
                                * BURN_TIME * BURN_TIME - DELAY_DROP;
  triggerPyro(P1);
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
  while (altitude > DELTA_ALT) sensors();
  triggerPyro(P2);  // not connected
  LED(true, false, true);
  unsigned long start = millis();
  while (millis() - start < 3000) TVC();
  triggerPyro(P4);
  while (altitude > 0.5f || accel_z > 10 || vert_vel < -0.5f) TVC();
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);
  pinMode(P1, OUTPUT); pinMode(P2, OUTPUT); pinMode(P3, OUTPUT); pinMode(P4, OUTPUT);
  digitalWrite(P1, LOW); digitalWrite(P2, LOW); digitalWrite(P3, LOW); digitalWrite(P4, LOW);

  servoX.attach(3);
  servoY.attach(4);
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);

  LED(false, false, false);
  Serial.begin(115200);
  Wire.begin();

  if (!bmp.begin(0x76)) { Serial.println(F("BMP280 not found")); while (true); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);

  mpu6050.begin();
  SD.begin(SD_CS);
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop() {
  // ── Arm ──
  LED(true, true, true); delay(500); LED(false, false, false);
  while (!buttonCount()) delay(50);
  delay(500);
  countdown();

  // ── Ascent ──
  unsigned long launchTime = millis();
  LED(true, true, false);
  poweredFlight = true;
  while (altitude > highest_alt - 1) {
    if (millis() - launchTime < (BURN_TIME + IGN_DELAY) * 1000) {
      TVC();
    } else {
      poweredFlight = false;   // burnout — emergency deploy disabled from here on
      sensors();
      servoX.write(90 + XTUNE);
      servoY.write(90 + YTUNE);
    }
  }
  poweredFlight = false;

  // ── Apogee ──
  beep(659, 100); beep(523, 100); beep(659, 100);
  LED(false, true, true);
  triggerPyro(P1);   // streamer at 1m below apogee

  // ── Descent ──
  float chuteAlt = highest_alt * 0.65f;
  while (altitude > chuteAlt) sensors();
  triggerPyro(P4);   // parachute at 65% of apogee altitude

  // ── Landed ──
  LED(true, true, true);
  Serial.println(F("LANDED"));
  while (digitalRead(BUTTON) == LOW) {
    beep(523, 1000);
    beep(392, 1000);
    updatePyros();
  }
  LED(false, false, false);
  while (true) updatePyros();
}
