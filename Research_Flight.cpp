/*
  Research_Flight.cpp — Sysiphus Rocket Control Robustness Research
  ------------------------------------------------------------------
  Based on Ascent_Test.cpp. Motor: F15 Estes.

  EXPERIMENT A — Actuator Delay
    Fixed initial condition: 15° (use calibrated wedge on launch rod)
    Variable: INJECTED_DELAY_MS = 0, 20, 40, 60, 80, 100
    Question: how does recovery degrade as delay increases?
    Trials: 3x baseline (0ms), 1x all others, per controller

  EXPERIMENT B — Initial Condition Recovery
    Fixed delay: INJECTED_DELAY_MS = 0
    Variable: launch angle set physically via wedge = 0, 5, 10, 15, 20°
    Question: what is the maximum recoverable disturbance per controller?
    Trials: 1x per condition per controller (0° and 15° shared with Exp A)
    NOTE: set INJECTED_DELAY_MS = 0 for all Exp B flights

  PRE-LAUNCH SEQUENCE:
    Boot → pad angle indicator → ARM (5 rapid presses) → countdown → ignite

  PAD ANGLE INDICATOR (after arm, before countdown):
    GREEN  = tilt < 5°   — vertical / near-vertical
    YELLOW = tilt 5–15°  — intermediate angle
    RED    = tilt > 15°  — high-angle condition
    Confirm actual angle from serial log (first GyroX/Y sample after ignition).
    Press button to proceed once at target angle.

  FILE NAMING — rename after each flight:
    Exp A: A_[ctrl]_delay[N]_T[trial].CSV   e.g. A_PD_delay40_T1.CSV
    Exp B: B_[ctrl]_angle[N]_T1.CSV         e.g. B_LQR_angle10_T1.CSV

  CSV columns: Time, Controller, DelayMS, Altitude, VertVel,
               GyroX, GyroY, GyroZ, AngVelX, AngVelY,
               AccelX, AccelY, AccelZ, ServoX, ServoY,
               Saturated, ComputeTime_us
  Files named RES000.CSV, RES001.CSV, ...
  ------------------------------------------------------------------
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ── Experiment Config (set before each upload) ────────────────────────────────
// ACTIVE_CONTROLLER  — pick one: CTRL_PD, CTRL_PID, CTRL_LQR
// INJECTED_DELAY_MS  — Exp A variable  (0, 20, 40, 60, 80, 100)
// TARGET_ANGLE       — physical launch-rod angle (deg)
//                      Exp A: always 15°   |   Exp B: 0, 5, 10, 15, 20°
#define              ACTIVE_CONTROLLER   CTRL_PD
constexpr float      INJECTED_DELAY_MS = 0;    // ms
constexpr float      TARGET_ANGLE      = 15.0; // deg

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

// ── PD Gains ──────────────────────────────────────────────────────────────────
constexpr float PD_P = 0.15, PD_D = 0.1;

// ── PID Gains ─────────────────────────────────────────────────────────────────
constexpr float PID_P    = 0.15, PID_I = 0.02, PID_D = 0.1;
constexpr float PID_IMAX = 10.0;   // anti-windup clamp (deg·s)

// ── LQR Gains ─────────────────────────────────────────────────────────────────
// Compute offline with scipy before first LQR flight:
//   import numpy as np
//   from scipy.linalg import solve_continuous_are
//   T=14.34; L=0.15; I=0.05   # thrust(N), CG-to-gimbal(m), MOI(kg·m²)
//   A=np.array([[0,1],[0,0]]); B=np.array([[0],[T*L/I]])
//   Q=np.diag([1.0, 0.1]); R=np.array([[1.0]])
//   P=solve_continuous_are(A,B,Q,R); K=(np.linalg.inv(R)@B.T@P).flatten()
//   print(f"LQR_K_ANGLE={K[0]:.4f}  LQR_K_RATE={K[1]:.4f}")
// PLACEHOLDER — replace with computed values before flying LQR.
constexpr float LQR_K_ANGLE = 0.15;
constexpr float LQR_K_RATE  = 0.10;

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

// ── Controller Type ───────────────────────────────────────────────────────────
enum ControllerType { CTRL_PD, CTRL_PID, CTRL_LQR };
ControllerType activeController = CTRL_PD;
const char* ctrlName[] = { "PD", "PID", "LQR" };

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
bool  poweredFlight = false;
bool  inFlight      = false;

// ── PID State ─────────────────────────────────────────────────────────────────
float         pid_integral_x = 0, pid_integral_y = 0;
unsigned long pid_last_t = 0;

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
    logFile.println(F("Time(ms),Controller,DelayMS,"
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
    "%lu,%s,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%lu",
    millis(), ctrlName[activeController],
    INJECTED_DELAY_MS,
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
  static unsigned long prevTime  = millis();
  static unsigned long angleTime = millis();
  static float prev_alt = 0;

  mpu6050.update();

  float raw_avx =   mpu6050.getGyroX();
  float raw_avy =  -mpu6050.getGyroY();
  ang_vel_x = ANGVEL_ALPHA * raw_avx + (1 - ANGVEL_ALPHA) * ang_vel_x;
  ang_vel_y = ANGVEL_ALPHA * raw_avy + (1 - ANGVEL_ALPHA) * ang_vel_y;

  // No sensor fusion — one source at a time:
  //   Ground: pure accelerometer atan2 (gravity = reliable reference)
  //   In air: pure gyro integration (accel reads thrust+gravity, unusable)
  unsigned long angleNow = millis();
  float dt = (angleNow - angleTime) / 1000.0f;
  angleTime = angleNow;
  if (inFlight && dt > 0 && dt < 0.1f) {
    gyro_x += ang_vel_x * dt;
    gyro_y += ang_vel_y * dt;
  } else if (!inFlight) {
    gyro_x = mpu6050.getAccAngleX();
    gyro_y = mpu6050.getAccAngleY();
  }
  // Z uses library integrated gyro (no accel reference for yaw)
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

// ── Controllers ───────────────────────────────────────────────────────────────
void computeController(float &outX, float &outY, bool &saturated) {
  float rawX, rawY;

  if (activeController == CTRL_PD) {
    rawX = PD_P * gyro_x + PD_D * ang_vel_x;
    rawY = PD_P * gyro_y + PD_D * ang_vel_y;

  } else if (activeController == CTRL_PID) {
    unsigned long now = millis();
    float dt = (now - pid_last_t) / 1000.0f;
    pid_last_t = now;
    if (dt > 0 && dt < 0.5f) {
      pid_integral_x += gyro_x * dt;
      pid_integral_y += gyro_y * dt;
    }
    pid_integral_x = constrain(pid_integral_x, -PID_IMAX, PID_IMAX);  // anti-windup
    pid_integral_y = constrain(pid_integral_y, -PID_IMAX, PID_IMAX);
    rawX = PID_P * gyro_x + PID_I * pid_integral_x + PID_D * ang_vel_x;
    rawY = PID_P * gyro_y + PID_I * pid_integral_y + PID_D * ang_vel_y;

  } else {   // LQR: u = -K*x, x = [angle, angular_rate]
    rawX = LQR_K_ANGLE * gyro_x + LQR_K_RATE * ang_vel_x;
    rawY = LQR_K_ANGLE * gyro_y + LQR_K_RATE * ang_vel_y;
  }

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

  if (INJECTED_DELAY_MS > 0) {
    pushServoCmd(cx, cy);
    float dx = cx, dy = cy;
    if (popServoCmd(dx, dy)) {
      servoX.write(-dx + 90 + XTUNE);
      servoY.write(-dy + 90 + YTUNE);
    }
    // Hold neutral while the delay buffer is filling on first loop
  } else {
    servoX.write(-cx + 90 + XTUNE);
    servoY.write(-cy + 90 + YTUNE);
  }
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

  Serial.print(F("Controller : ")); Serial.println(ctrlName[activeController]);
  Serial.print(F("Delay (ms) : ")); Serial.println(INJECTED_DELAY_MS);

  // Gyro calibration: rocket is still on the pad during countdown, so the
  // integrated gyro angle IS the accumulated drift. Drift / time = bias.
  float angle_x = 0, angle_y = 0;
  unsigned long calStart = millis();
  unsigned long calTime  = calStart;

  for (int i = DURATION; i > 0; i--) {
    Serial.println(i);
    unsigned long secStart = millis();

    if (i > 5) {
      LED(true, false, false); beep(440, 200); LED(false, false, false);
    } else if (i > 2) {
      LED(true, false, false); tone(BUZZER, 440);
    }

    if (i > 2) {
      calTime = millis();   // reset after beep() so first dt is valid
      while (millis() - secStart < 1000) {
        unsigned long prev = calTime;
        mpu6050.update();
        calTime = millis();
        float dt = (calTime - prev) / 1000.0f;
        if (dt > 0 && dt < 0.1f) {
          angle_x += mpu6050.getGyroX() * dt;
          angle_y += mpu6050.getGyroY() * dt;
        }
      }
    }
    noTone(BUZZER); LED(false, false, false);
  }

  // Average rate over the countdown = gyro bias
  float elapsed = (calTime - calStart) / 1000.0f;
  if (elapsed < 1) elapsed = 28.0f;
  float offX = angle_x / elapsed;
  float offY = angle_y / elapsed;
  mpu6050.setGyroOffsets(offX, offY, 0.0f);

  Serial.print(F("Gyro drift angle: X=")); Serial.print(angle_x, 2);
  Serial.print(F("  Y=")); Serial.println(angle_y, 2);
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

  // Initialize angle state from atan ground truth — gyro integration
  // will continue from this point once inFlight goes true
  mpu6050.update();
  gyro_x = mpu6050.getAccAngleX();
  gyro_y = mpu6050.getAccAngleY();
  gyro_z = 0;
  ang_vel_x = ang_vel_y = 0;
  pid_integral_x = pid_integral_y = 0;
  pid_last_t = millis();
  delayBuf.head = delayBuf.count = 0;

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

  servoX.attach(3);
  servoY.attach(4);
  servoX.write(90 + XTUNE);
  servoY.write(90 + YTUNE);

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

  activeController = ACTIVE_CONTROLLER;

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║  SYSIPHUS RESEARCH FLIGHT v1.0   ║"));
  Serial.println(F("╚══════════════════════════════════╝"));
  Serial.print(F("Controller  : ")); Serial.println(ctrlName[activeController]);
  Serial.print(F("Delay       : ")); Serial.print(INJECTED_DELAY_MS); Serial.println(F(" ms"));
  Serial.print(F("Target angle: ")); Serial.print(TARGET_ANGLE);      Serial.println(F(" deg"));

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
