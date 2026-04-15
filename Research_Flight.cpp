/*
  Research_Flight.cpp — Sysiphus Rocket Stability-Boundary Study
  ------------------------------------------------------------------
  Motor: F15 Estes. Fixed proportional–derivative (PD) controller.

  Purpose: experimentally map the stability boundary of a small-scale
  TVC rocket as a joint function of (1) initial tilt angle, (2) feedback
  delay, and (3) actuator slew rate. Outcome per flight = RECOVER or
  DIVERGE (+ failure mode: lag-divergence vs. limit cycle).

  EXPERIMENT VARIABLES (set before each upload):
    TARGET_ANGLE        — initial tilt set by physical launch-rod wedge
                          sweep: 0, 5, 10, 15, 20 deg
    INJECTED_DELAY_MS   — artificial feedback delay inserted in the
                          actuator command path
                          sweep: 0, 20, 40, 60, 80, 100 ms
    SLEW_RATE_LIMIT_DPS — software-imposed servo slew cap
                          sweep: 0 (unlimited), 600, 400, 200, 100 dps

  PRE-LAUNCH SEQUENCE:
    Boot → pad angle indicator → ARM (5 rapid presses) → countdown → ignite

  PAD ANGLE INDICATOR (after arm, before countdown):
    GREEN  = |tilt - TARGET_ANGLE| < 1°   on target
    BLUE   = tilt < TARGET_ANGLE - 1°     too vertical
    RED    = tilt > TARGET_ANGLE + 1°     too tilted

  FILE NAMING — rename RES###.CSV after each flight:
    angle[N]_delay[N]_slew[N]_T[trial].CSV
    e.g. angle15_delay40_slew400_T1.CSV

  CSV columns: Time, DelayMS, SlewDPS,
               Altitude, VertVel,
               GyroX, GyroY, GyroZ, AngVelX, AngVelY,
               AccelX, AccelY, AccelZ, ServoX, ServoY,
               Saturated, ComputeTime_us
  ------------------------------------------------------------------
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ── Experiment Config (set before each upload) ────────────────────────────────
// TARGET_ANGLE         — initial tilt, set physically via launch-rod wedge (deg)
// INJECTED_DELAY_MS    — feedback delay inserted in actuator command path (ms)
// SLEW_RATE_LIMIT_DPS  — servo slew-rate cap (deg/s); 0 = unlimited
constexpr float      TARGET_ANGLE        = 0.0;   // deg
constexpr float      INJECTED_DELAY_MS   = 0;     // ms
constexpr float      SLEW_RATE_LIMIT_DPS = 0;     // deg/s (servo output rate)

// ── Pins ──────────────────────────────────────────────────────────────────────
constexpr int BUTTON = 14;
constexpr int BUZZER = 13;
constexpr int P1     = 9;    // Landing legs
constexpr int P2     = 10;   // Not connected
constexpr int P3     = 11;   // Ascent motor ignition
constexpr int P4     = 12;   // Parachute
constexpr int RLED   = 6;
constexpr int GLED   = 7;
constexpr int BLED   = 8;
constexpr int SD_CS  = BUILTIN_SDCARD;

// ── Tuning ────────────────────────────────────────────────────────────────────
constexpr float XTUNE        = 0, YTUNE = 0;
constexpr float SERVO_X_MULT = 8.13;
constexpr float SERVO_Y_MULT = 4.33;
constexpr float MAX_TILT     = 5;      // degrees, nozzle deflection limit

// ── PD Gains (fixed for all flights — this study holds the controller constant
//              and varies delay, slew rate, and initial tilt only) ────────────
constexpr float PD_P = 0.08, PD_D = 0.08;

// ── Motor & Flight Constants ──────────────────────────────────────────────────
constexpr float BURN_TIME     = 3.45;
constexpr float AV_THRUST     = 14.34;
constexpr float ROCKET_WEIGHT = 0.78;
constexpr float G             = 9.81;
constexpr float IGN_DELAY     = 0.2;
constexpr float SEA_LEVEL_HPA = 1013.25;

// ── Sensor Smoothing ──────────────────────────────────────────────────────────
constexpr float ANGVEL_ALPHA = 0.9;
constexpr float VEL_ALPHA    = 0.2;

// ── Hardware ──────────────────────────────────────────────────────────────────
PWMServo        servoX, servoY;
MPU6050         mpu6050(Wire);
Adafruit_BMP280 bmp;
File            logFile;
char            logFilename[20];

// ── Flight State ──────────────────────────────────────────────────────────────
float altitude, initial_alt, highest_alt, vert_vel;
float gyro_x, gyro_y, gyro_z;
float ang_vel_x, ang_vel_y;
float accel_x, accel_y, accel_z;
float tiltX, tiltY;
float padAngle_x = 0, padAngle_y = 0;    // atan angle captured at launch
float gyroRef_x  = 0, gyroRef_y  = 0;    // library gyro angle at launch
bool  poweredFlight = false;
bool  inFlight      = false;

// ── Slew-Rate Limiter State ───────────────────────────────────────────────────
// Tracks the previous rate-limited servo command. Applied AFTER the delay
// buffer so the limit reflects what the actuator can physically achieve.
float         slew_last_x = 0, slew_last_y = 0;
unsigned long slew_last_t = 0;

float applySlewLimit(float target, float &last, float dt) {
  if (SLEW_RATE_LIMIT_DPS <= 0) { last = target; return target; }
  float max_delta = SLEW_RATE_LIMIT_DPS * dt;
  last += constrain(target - last, -max_delta, max_delta);
  return last;
}

// ── Actuator Delay Buffer ─────────────────────────────────────────────────────
// Holds up to 60 commands — enough for 100ms delay at a ~10ms loop rate
constexpr int DELAY_BUF_SIZE = 60;
struct {
  float         x[DELAY_BUF_SIZE], y[DELAY_BUF_SIZE];
  unsigned long t[DELAY_BUF_SIZE];
  int head  = 0;
  int count = 0;
} delayBuf;

// ── Research Metrics (logged each cycle) ─────────────────────────────────────
bool          last_saturated    = false;
unsigned long last_compute_us   = 0; 

// ── Pyro System ───────────────────────────────────────────────────────────────
struct PyroChannel { int pin; bool active; unsigned long startTime; };
PyroChannel pyros[] = {{P1,false,0},{P2,false,0},{P3,false,0},{P4,false,0}};

void triggerPyro(int pin) {
  for (auto& ch : pyros)
    if (ch.pin == pin && !ch.active) {
      digitalWrite(pin, HIGH);
      ch.active = true; ch.startTime = millis();
    }
}

void updatePyros() {
  for (auto& ch : pyros)
    if (ch.active && millis() - ch.startTime >= 1000) {
      digitalWrite(ch.pin, LOW); ch.active = false;
    }
}

// ── Utilities ─────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r); digitalWrite(GLED, !g); digitalWrite(BLED, !b);
}

void beep(int freq, int dur) { tone(BUZZER, freq); delay(dur); noTone(BUZZER); }


// ── Actuator Delay Buffer ─────────────────────────────────────────────────────
void pushServoCmd(float cx, float cy) {
  delayBuf.x[delayBuf.head] = cx;
  delayBuf.y[delayBuf.head] = cy;
  delayBuf.t[delayBuf.head] = millis();
  delayBuf.head = (delayBuf.head + 1) % DELAY_BUF_SIZE;
  if (delayBuf.count < DELAY_BUF_SIZE) delayBuf.count++;
}

// Returns true and fills cx/cy when the oldest command has aged past INJECTED_DELAY_MS
bool popServoCmd(float &cx, float &cy) {
  if (delayBuf.count == 0) return false;
  int tail = ((delayBuf.head - delayBuf.count) % DELAY_BUF_SIZE + DELAY_BUF_SIZE) % DELAY_BUF_SIZE;
  if (millis() - delayBuf.t[tail] < (unsigned long)INJECTED_DELAY_MS) return false;
  cx = delayBuf.x[tail];
  cy = delayBuf.y[tail];
  delayBuf.count--;
  return true;
}

// ── SD Logging ────────────────────────────────────────────────────────────────
void createUniqueLogFile() {
  int idx = 0;
  do { sprintf(logFilename, "RES%03d.CSV", idx++); }
  while (SD.exists(logFilename) && idx < 1000);

  logFile = SD.open(logFilename, FILE_WRITE);
  if (logFile) {
    Serial.print(F("Logging to: ")); Serial.println(logFilename);
    logFile.println(F("Time(ms),DelayMS,SlewDPS,"
                      "Altitude(m),VertVel(m/s),"
                      "GyroX,GyroY,GyroZ,AngVelX,AngVelY,"
                      "AccelX,AccelY,AccelZ,"
                      "ServoX,ServoY,Saturated,ComputeTime_us"));
    logFile.close();
  } else {
    Serial.println(F("Failed to create log file!"));
  }
}

void logData() {
  logFile = SD.open(logFilename, FILE_WRITE);
  if (!logFile) return;
  char buf[176];
  snprintf(buf, sizeof(buf),
    "%lu,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%lu",
    millis(),
    INJECTED_DELAY_MS, SLEW_RATE_LIMIT_DPS,
    altitude, vert_vel,
    gyro_x, gyro_y, gyro_z,
    ang_vel_x, ang_vel_y,
    accel_x, accel_y, accel_z,
    constrain(tiltX, -MAX_TILT, MAX_TILT),
    constrain(tiltY, -MAX_TILT, MAX_TILT),
    (int)last_saturated, last_compute_us);
  logFile.println(buf);
  logFile.close();
}

// ── Sensors ───────────────────────────────────────────────────────────────────
void sensors() {
  static unsigned long prevTime = millis();
  static float prev_alt = 0;

  mpu6050.update();

  float raw_avx =  mpu6050.getGyroX();
  float raw_avy = -mpu6050.getGyroY();   // Y inverted (sensor mounting)
  ang_vel_x = ANGVEL_ALPHA * raw_avx + (1 - ANGVEL_ALPHA) * ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA * raw_avy + (1 - ANGVEL_ALPHA) * ang_vel_y;

  // No sensor fusion — one source at a time:
  //   Ground: accel atan2 (gravity = reliable reference)
  //   In air: library gyro-angle delta from launch snapshot + pad angle
  if (inFlight) {
    gyro_x = padAngle_x + (mpu6050.getGyroAngleX() - gyroRef_x);
    gyro_y = padAngle_y - (mpu6050.getGyroAngleY() - gyroRef_y);   // Y inverted
  } else {
    gyro_x =  mpu6050.getAccAngleX();
    gyro_y = -mpu6050.getAccAngleY();   // Y inverted
  }
  gyro_z = mpu6050.getGyroAngleZ();

  accel_x = mpu6050.getAccX() * G;
  accel_y = mpu6050.getAccY() * G;
  accel_z = mpu6050.getAccZ() * G;

  altitude = bmp.readAltitude(SEA_LEVEL_HPA) - initial_alt;
  if (altitude > highest_alt) highest_alt = altitude;

  unsigned long now = millis();
  unsigned long elapsed = now - prevTime;
  if (elapsed >= 50) {
    float raw_vel = (altitude - prev_alt) / (elapsed / 1000.0f);
    vert_vel = VEL_ALPHA * raw_vel + (1 - VEL_ALPHA) * vert_vel;
    prev_alt = altitude;
    prevTime = now;
    logData();
  }

  // Flight gyro debug — print every 100ms while in air
  static unsigned long dbgTime = 0;
  if (inFlight && now - dbgTime >= 100) {
    dbgTime = now;
    Serial.print(F("t=")); Serial.print(now);
    Serial.print(F(" gx=")); Serial.print(gyro_x, 1);
    Serial.print(F(" gy=")); Serial.print(gyro_y, 1);
    Serial.print(F(" gz=")); Serial.print(gyro_z, 1);
    Serial.print(F(" avx=")); Serial.print(ang_vel_x, 1);
    Serial.print(F(" avy=")); Serial.print(ang_vel_y, 1);
    Serial.print(F(" alt=")); Serial.println(altitude, 1);
  }

  updatePyros();
  emergency();
}

// ── Emergency ─────────────────────────────────────────────────────────────────
void emergency() {
  if (!poweredFlight) return;
  if (abs(gyro_x) <= 45 && abs(gyro_y) <= 45) return;

  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
  Serial.println(F("EMERGENCY"));
  LED(true, false, false);
  triggerPyro(P4);
  delay(1000);
  triggerPyro(P1);
  while (true) {
    updatePyros();
    beep(500, 50); delay(50);
    if (digitalRead(BUTTON) == HIGH) while (true) updatePyros();
  }
}

// ── Controller (fixed PD) ─────────────────────────────────────────────────────
void computeController(float &outX, float &outY, bool &saturated) {
  float rawX = PD_P * gyro_x + PD_D * ang_vel_x;
  float rawY = PD_P * gyro_y + PD_D * ang_vel_y;

  saturated = (abs(rawX) > MAX_TILT || abs(rawY) > MAX_TILT);
  outX = constrain(rawX, -MAX_TILT, MAX_TILT) * SERVO_X_MULT;
  outY = constrain(rawY, -MAX_TILT, MAX_TILT) * SERVO_Y_MULT;
}

// ── TVC (with optional actuator delay — Experiment A) ────────────────────────
void TVC() {
  unsigned long t0 = micros();
  sensors();

  bool  sat = false;
  float cx, cy;
  computeController(cx, cy, sat);

  last_saturated  = sat;
  last_compute_us = micros() - t0;

  // Pull the controller output through the delay buffer (if any), then through
  // the slew-rate limiter, then to the servo. Chain order: controller → delay
  // → slew → servo mirrors the physical signal path in a rate-limited system.
  float targetX = cx, targetY = cy;
  if (INJECTED_DELAY_MS > 0) {
    pushServoCmd(cx, cy);
    if (!popServoCmd(targetX, targetY)) return;   // buffer still filling
  }

  unsigned long now = millis();
  float dt = (now - slew_last_t) / 1000.0f;
  slew_last_t = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;
  float ax = applySlewLimit(targetX, slew_last_x, dt);
  float ay = applySlewLimit(targetY, slew_last_y, dt);

  servoX.write(-ax + 90 + XTUNE);
  servoY.write(-ay + 90 + YTUNE);
}

// ── Arm ───────────────────────────────────────────────────────────────────────
bool buttonCount() {
  constexpr int PRESS_WINDOW = 300;
  constexpr int PRESSES_REQD = 5;
  static unsigned long lastPress = 0;
  static int pressCount = 0;
  if (digitalRead(BUTTON) == HIGH) {
    if (millis() - lastPress <= PRESS_WINDOW) {
      pressCount++;
      LED(false, true, false);
      tone(BUZZER, 311.13f * pow(2.0f, pressCount / 12.0f));
      while (digitalRead(BUTTON) == HIGH) {}
      noTone(BUZZER); LED(false, false, false);
    } else { pressCount = 1; }
    lastPress = millis();
  }
  return pressCount > PRESSES_REQD;
}

// ── Pad Angle Indicator ───────────────────────────────────────────────────────
// Fly-the-ball: guides rocket to TARGET_ANGLE like a carrier landing glideslope.
// Tilt is computed from the accelerometer (atan2 of lateral vs vertical g-force)
// so it reads true 0° when vertical and is independent of gyro calibration state.
//   BLUE  = below target  (tilt < TARGET - 1°) — tilt rocket more
//   GREEN = on target     (tilt within ±1° of TARGET_ANGLE)
//   RED   = past target   (tilt > TARGET + 1°) — tilt rocket back toward vertical
// Press button when GREEN to lock in angle and start countdown.
void padAngleIndicator() {
  Serial.print(F("\nPAD ANGLE — target: "));
  Serial.print(TARGET_ANGLE, 1);
  Serial.println(F("°"));
  Serial.println(F("  BLUE=below target  GREEN=on target  RED=past target"));

  while (true) {
    mpu6050.update();
    float gx   = mpu6050.getAccX();
    float gy   = mpu6050.getAccY();
    float gz   = mpu6050.getAccZ();
    float tilt = atan2f(sqrtf(gx*gx + gy*gy), gz) * 180.0f / M_PI;
    float err  = tilt - TARGET_ANGLE;

    if      (err < -1.0f) LED(false, false, true);    // BLUE  — too vertical
    else if (err >  1.0f) LED(true,  false, false);   // RED   — too horizontal
    else                  LED(false, true,  false);   // GREEN — on target

    Serial.print(F("  tilt=")); Serial.print(tilt, 1);
    Serial.print(F("°  target=")); Serial.print(TARGET_ANGLE, 1);
    Serial.print(F("°  err=")); Serial.print(err, 1);
    Serial.println(F("°"));

    if (digitalRead(BUTTON) == HIGH) {
      delay(30);
      while (digitalRead(BUTTON) == HIGH) delay(10);
      if (abs(err) > 1.0f) {
        // Not on target — warn and require re-confirm
        LED(true, true, false);
        beep(440, 80); delay(80); beep(440, 80);
        Serial.println(F("  WARNING: confirmed off-target. Press again to accept anyway."));
        unsigned long warn = millis();
        while (digitalRead(BUTTON) == LOW && millis() - warn < 3000) delay(20);
        if (digitalRead(BUTTON) == LOW) continue;   // timed out — keep adjusting
        while (digitalRead(BUTTON) == HIGH) delay(10);
      }
      break;
    }
    delay(100);
  }
  LED(false, false, false);
  Serial.print(F("  Angle locked. Starting countdown.\n"));
}

// ── Countdown & Launch ────────────────────────────────────────────────────────
bool countdown() {
  constexpr int DURATION = 30;
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);
  createUniqueLogFile();

  Serial.print(F("Target angle (deg): ")); Serial.println(TARGET_ANGLE);
  Serial.print(F("Delay (ms)        : ")); Serial.println(INJECTED_DELAY_MS);
  Serial.print(F("Slew limit (dps)  : ")); Serial.println(SLEW_RATE_LIMIT_DPS);

  // Gyro calibration: rocket is still on pad. Library auto-integrates gyro
  // on every update(); read start and end angles, drift / time = bias.
  mpu6050.update();
  float start_x = mpu6050.getGyroAngleX();
  float start_y = mpu6050.getGyroAngleY();
  unsigned long t0 = millis();

  for (int i = DURATION; i > 0; i--) {
    Serial.println(i);
    unsigned long secStart = millis();

    if (i > 5) {
      LED(true, false, false); beep(440, 200); LED(false, false, false);
    } else if (i > 2) {
      LED(true, false, false); tone(BUZZER, 440);
    }
    while (millis() - secStart < 1000) mpu6050.update();
    noTone(BUZZER); LED(false, false, false);
  }

  float elapsed = (millis() - t0) / 1000.0f;
  float drift_x = mpu6050.getGyroAngleX() - start_x;
  float drift_y = mpu6050.getGyroAngleY() - start_y;
  float offX = drift_x / elapsed;
  float offY = drift_y / elapsed;
  mpu6050.setGyroOffsets(offX, offY, 0.0f);

  Serial.print(F("Elapsed: ")); Serial.print(elapsed, 2); Serial.println(F(" s"));
  Serial.print(F("Drift angle: X=")); Serial.print(drift_x, 2);
  Serial.print(F("  Y=")); Serial.println(drift_y, 2);
  Serial.print(F("Offset (dps): X=")); Serial.print(offX, 3);
  Serial.print(F("  Y=")); Serial.println(offY, 3);

  // Refresh sensor data with new offsets applied
  for (int i = 0; i < 10; i++) { mpu6050.update(); delay(10); }

  // Abort — atan angle ±2° off target OR angular rate > 10 dps
  float gx = mpu6050.getAccX(), gy = mpu6050.getAccY(), gz = mpu6050.getAccZ();
  float tilt = atan2f(sqrtf(gx*gx + gy*gy), gz) * 180.0f / M_PI;
  float rate_x = mpu6050.getGyroX(), rate_y = mpu6050.getGyroY();
  bool bad_angle = abs(tilt - TARGET_ANGLE) > 2.0f;
  bool bad_rate  = abs(rate_x) > 10.0f || abs(rate_y) > 10.0f;

  if (bad_angle || bad_rate) {
    Serial.print(F("ABORT — "));
    if (bad_angle) { Serial.print(F("angle=")); Serial.print(tilt, 1); Serial.print(F("° ")); }
    if (bad_rate)  { Serial.print(F("rate=")); Serial.print(rate_x, 1); Serial.print(F(",")); Serial.print(rate_y, 1); Serial.print(F(" dps")); }
    Serial.println();
    LED(true, false, false);
    for (int j = 0; j < 10; j++) { beep(880, 100); delay(100); }
    LED(false, false, false);
    return false;
  }

  // Snapshot pad angle (atan ground truth) and library gyro angle at launch.
  // In-flight: gyro_x = padAngle_x + (getGyroAngleX() - gyroRef_x)
  mpu6050.update();
  gyro_x      =  mpu6050.getAccAngleX();
  gyro_y      = -mpu6050.getAccAngleY();   // Y inverted (match sensors())
  padAngle_x  = gyro_x;
  padAngle_y  = gyro_y;
  gyroRef_x   = mpu6050.getGyroAngleX();
  gyroRef_y   = mpu6050.getGyroAngleY();
  gyro_z      = 0;
  ang_vel_x = ang_vel_y = 0;
  delayBuf.head = delayBuf.count = 0;
  slew_last_x = slew_last_y = 0;
  slew_last_t = millis();

  initial_alt = bmp.readAltitude(SEA_LEVEL_HPA);
  Serial.print(F("  baseline alt: ")); Serial.println(initial_alt);
  Serial.println(F("LAUNCH"));
  triggerPyro(P3);
  return true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);
  digitalWrite(RLED, HIGH); digitalWrite(GLED, HIGH); digitalWrite(BLED, HIGH);
  pinMode(P1, OUTPUT); pinMode(P2, OUTPUT); pinMode(P3, OUTPUT); pinMode(P4, OUTPUT);
  digitalWrite(P1, LOW); digitalWrite(P2, LOW); digitalWrite(P3, LOW); digitalWrite(P4, LOW);

  Serial.begin(115200);
  Wire.begin();

  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println(F("BMP280 not found"));
    while (true);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
  mpu6050.begin();
  Wire.setClock(400000);   // 400 kHz I2C after init — higher sample rate
  SD.begin(SD_CS);

  // Servos init after I2C is up — attaching before Wire.begin() has been
  // observed to interfere with BMP280 detection on this board.
  servoX.attach(4);
  servoY.attach(3);
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║  SYSIPHUS RESEARCH FLIGHT v1.0   ║"));
  Serial.println(F("╚══════════════════════════════════╝"));
  Serial.println(F("Fixed PD controller — stability-boundary study"));
  Serial.print(F("Target angle: ")); Serial.print(TARGET_ANGLE);        Serial.println(F(" deg"));
  Serial.print(F("Delay       : ")); Serial.print(INJECTED_DELAY_MS);   Serial.println(F(" ms"));
  Serial.print(F("Slew limit  : ")); Serial.print(SLEW_RATE_LIMIT_DPS); Serial.println(F(" dps (0=unlimited)"));

  beep(523, 80); beep(659, 80); beep(784, 120);
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop() {
  // Pad angle indicator
  padAngleIndicator();
  delay(500);

  // Arm
  LED(true, true, true); delay(500); LED(false, false, false);
  Serial.println(F("ARM — 5 rapid presses"));
  while (!buttonCount()) delay(50);
  beep(880, 150);

  // Countdown + launch abort check
  if (!countdown()) {
    LED(true, false, false);
    while (digitalRead(BUTTON) == LOW) delay(50);
    LED(false, false, false);
    return;
  }

  // ── Ascent ──
  unsigned long launchTime = millis();
  LED(true, true, false);
  poweredFlight = true;
  inFlight = true;
  while (altitude > highest_alt - 1) {
    if (millis() - launchTime < (BURN_TIME + IGN_DELAY) * 1000) {
      TVC();
    } else {
      poweredFlight = false;
      sensors();
      servoX.write(90 + XTUNE);
      servoY.write(90 + YTUNE);
    }
  }
  poweredFlight = false;
  inFlight = false;

  // ── Apogee ──
  beep(659, 100); beep(523, 100); beep(659, 100);
  LED(false, true, true);
  triggerPyro(P4);
  triggerPyro(P1);

  // ── Landed ──
  LED(true, true, true);
  Serial.println(F("LANDED"));
  while (digitalRead(BUTTON) == LOW) {
    beep(523, 1000); beep(392, 1000); updatePyros();
  }
  LED(false, false, false);
  while (true) updatePyros();
}
