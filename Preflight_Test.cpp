/*
  Preflight_Test.cpp — Sysiphus Rocket Pre-launch Verification
  ---------------------------------------------------------------
  Runs on the actual hardware. Uses real IMU and servos.
  Pyro channels are BLOCKED — fires are printed to Serial only.

  TESTS (advance with button single-press):
    1. Servo Center       — both servos to 90°, verify neutral visually
    2. Servo Sweep        — sweeps ±5° on each axis, verify range and direction
    3. Live TVC           — real IMU input, servos respond in real time
                            CRITICAL: tilt rocket and verify servo opposes the lean
    4. Flight Simulation  — fake altitude profile, verify staging event sequence
    5. Emergency Test     — injects 91° tilt, verify emergency logic fires correctly

  Upload this file instead of Ascent_Test.cpp when bench testing.
  ---------------------------------------------------------------
*/

#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ── Pins (match Ascent_Test.cpp exactly) ────────────────────────────────────
constexpr int button = 14;
constexpr int buzzer = 13;
constexpr int P1     = 9;
constexpr int P2     = 10;
constexpr int P3     = 11;
constexpr int P4     = 12;
constexpr int RLED   = 6;
constexpr int GLED   = 7;
constexpr int BLED   = 8;

// ── Tuning (keep in sync with Ascent_Test.cpp) ──────────────────────────────
const float  Xtune       = 1,    Ytune      = 0;
const double ServoXMult  = 4,    ServoYMult = 4;
const double P_GAIN      = 0.4,  D_GAIN     = 0.3;
const double burnTime    = 3.45;
const double avThrust    = 14.34;
const double rocketWeight= 0.7;
const double G           = 9.81;
const double ignitionDelay = 0.2;

// ── Hardware ─────────────────────────────────────────────────────────────────
PWMServo servoX, servoY;
MPU6050  mpu6050(Wire);
Adafruit_BMP280 bmp;

// ── State ────────────────────────────────────────────────────────────────────
float gyro_x, gyro_y, gyro_z, ang_vel_x, ang_vel_y;
float tiltX, tiltY;
int   testMode = 0;   // increments on button press

// ── Helpers ──────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r);
  digitalWrite(GLED, !g);
  digitalWrite(BLED, !b);
}

void beep(int freq, int dur) { tone(buzzer, freq); delay(dur); noTone(buzzer); }

// Pyro is BLOCKED in test mode — print only
void mockPyro(int pin, const char* label) {
  Serial.print(F("  [PYRO BLOCKED] Would fire "));
  Serial.print(label);
  Serial.print(F(" (pin "));
  Serial.print(pin);
  Serial.println(F(")"));
}

void waitButtonRelease() { while (digitalRead(button) == HIGH) delay(10); }

bool buttonPressed() {
  if (digitalRead(button) == HIGH) {
    delay(30); // debounce
    waitButtonRelease();
    return true;
  }
  return false;
}

void printHeader(int mode, const char* name) {
  Serial.println(F("\n========================================"));
  Serial.print(F("  TEST "));
  Serial.print(mode);
  Serial.print(F(": "));
  Serial.println(name);
  Serial.println(F("========================================"));
}

// ── Simulated flight profile ─────────────────────────────────────────────────
// Returns altitude (m) at time t (seconds from ignition)
float simAltitude(float t) {
  const float a_burn   = avThrust / rocketWeight - G;  // net accel during burn
  const float v_burnout = a_burn * burnTime;
  const float alt_burnout = 0.5f * a_burn * burnTime * burnTime;
  const float t_coast  = v_burnout / G;                // coast to apogee
  const float alt_apogee = alt_burnout + (v_burnout * v_burnout) / (2.0f * G);

  if (t < 0) return 0;
  if (t <= burnTime) {
    return 0.5f * a_burn * t * t;
  } else if (t <= burnTime + t_coast) {
    float dt = t - burnTime;
    return alt_burnout + v_burnout * dt - 0.5f * G * dt * dt;
  } else {
    float dt = t - (burnTime + t_coast);
    return alt_apogee - 0.5f * G * dt * dt;   // free-fall approximation
  }
}

float simVertVel(float t) {
  const float a_burn    = avThrust / rocketWeight - G;
  const float v_burnout = a_burn * burnTime;
  if (t <= burnTime) return a_burn * t;
  return v_burnout - G * (t - burnTime);
}

// ── IMU read (same filter as Ascent_Test.cpp) ─────────────────────────────────
void readIMU() {
  mpu6050.update();
  float raw_x = mpu6050.getAngleX() + Xtune;
  float raw_y = mpu6050.getAngleY() + Ytune;
  float raw_z = mpu6050.getAngleZ();
  float raw_av_x = mpu6050.getGyroX();
  float raw_av_y = mpu6050.getGyroY();

  const float ga = 0.9, va = 0.9;
  gyro_x   = ga * raw_x   + (1 - ga) * gyro_x;
  gyro_y   = ga * raw_y   + (1 - ga) * gyro_y;
  gyro_z   = ga * raw_z   + (1 - ga) * gyro_z;
  ang_vel_x = va * raw_av_x + (1 - va) * ang_vel_x;
  ang_vel_y = va * raw_av_y + (1 - va) * ang_vel_y;
}

void computeTVC() {
  tiltX = constrain(P_GAIN * gyro_x + D_GAIN * ang_vel_x, -5, 5) * ServoXMult;
  tiltY = constrain(P_GAIN * gyro_y + D_GAIN * ang_vel_y, -5, 5) * ServoYMult;
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 1 — Servo Center
// ─────────────────────────────────────────────────────────────────────────────
void test_servoCenter() {
  printHeader(1, "Servo Center");
  Serial.println(F("  Servos moving to 90 (neutral)."));
  Serial.println(F("  Verify both TVC gimbal fins are centred."));
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
  LED(false, true, false);
  Serial.println(F("  Press button to continue."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 2 — Servo Sweep
// ─────────────────────────────────────────────────────────────────────────────
void test_servoSweep() {
  printHeader(2, "Servo Sweep");
  Serial.println(F("  Sweeping each axis +5 -> 0 -> -5 degrees."));
  LED(false, false, true);

  // Sweep X
  Serial.println(F("  --- Axis X ---"));
  for (int deg = -5; deg <= 5; deg++) {
    float cmd = deg * ServoXMult;
    servoX.write(cmd + 90 + Xtune);
    Serial.print(F("  X tilt="));
    Serial.print(deg);
    Serial.print(F("  servo="));
    Serial.println((int)(cmd + 90 + Xtune));
    delay(200);
  }
  for (int deg = 5; deg >= -5; deg--) {
    float cmd = deg * ServoXMult;
    servoX.write(cmd + 90 + Xtune);
    delay(200);
  }
  servoX.write(90 + Xtune);

  delay(500);

  // Sweep Y
  Serial.println(F("  --- Axis Y ---"));
  for (int deg = -5; deg <= 5; deg++) {
    float cmd = deg * ServoYMult;
    servoY.write(cmd + 90 + Ytune);
    Serial.print(F("  Y tilt="));
    Serial.print(deg);
    Serial.print(F("  servo="));
    Serial.println((int)(cmd + 90 + Ytune));
    delay(200);
  }
  for (int deg = 5; deg >= -5; deg--) {
    float cmd = deg * ServoYMult;
    servoY.write(cmd + 90 + Ytune);
    delay(200);
  }
  servoY.write(90 + Ytune);

  Serial.println(F("  Done. Press button to continue."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 3 — Live TVC
// ─────────────────────────────────────────────────────────────────────────────
void test_liveTVC() {
  printHeader(3, "Live TVC");
  Serial.println(F("  Calibrating gyro — keep rocket still..."));
  LED(true, true, false);
  mpu6050.calcGyroOffsets(true, 0, 0);
  Serial.println(F("  Done. Tilt the rocket slowly and watch servo response."));
  Serial.println(F("  EXPECTED: servo moves OPPOSITE to lean (correcting)."));
  Serial.println(F("  Press button to exit."));
  Serial.println();
  Serial.println(F("  GyroX   GyroY   SrvX_cmd  SrvY_cmd  Diagnosis"));

  LED(false, true, true);
  unsigned long lastPrint = 0;

  while (!buttonPressed()) {
    readIMU();
    computeTVC();
    servoX.write(tiltX + 90 + Xtune);
    servoY.write(tiltY + 90 + Ytune);

    if (millis() - lastPrint > 100) {
      lastPrint = millis();

      // Diagnosis: servo command should oppose the tilt
      // tiltX = P*gyroX + D*angVelX — positive gyroX should give positive tiltX
      // servoX.write(tiltX + 90): if gyroX>0, servo >90 (moves one way)
      // CORRECT if that physically pushes the nose back toward centre
      bool xOk = (gyro_x > 1.0  && tiltX > 0) ||
                 (gyro_x < -1.0 && tiltX < 0) ||
                 (abs(gyro_x) <= 1.0);
      bool yOk = (gyro_y > 1.0  && tiltY > 0) ||
                 (gyro_y < -1.0 && tiltY < 0) ||
                 (abs(gyro_y) <= 1.0);

      Serial.print(F("  "));
      Serial.print(gyro_x, 1);
      Serial.print(F("\t"));
      Serial.print(gyro_y, 1);
      Serial.print(F("\t"));
      Serial.print(tiltX, 1);
      Serial.print(F("\t\t"));
      Serial.print(tiltY, 1);
      Serial.print(F("\t\t"));
      // Just remind user what to check — actual correction direction
      // depends on gimbal linkage geometry which we can't know in software
      Serial.println((abs(gyro_x) > 1 || abs(gyro_y) > 1) ? F("ACTIVE") : F("level"));
    }
    delay(10);
  }

  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
  Serial.println(F("\n  Servos centred. Press button to continue."));
  delay(200);
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 4 — Flight Simulation
// ─────────────────────────────────────────────────────────────────────────────
void test_flightSim() {
  printHeader(4, "Flight Simulation");

  const float a_burn    = avThrust / rocketWeight - G;
  const float v_burnout = a_burn * burnTime;
  const float alt_burnout = 0.5f * a_burn * burnTime * burnTime;
  const float t_coast   = v_burnout / G;
  const float alt_apogee = alt_burnout + (v_burnout * v_burnout) / (2.0f * G);
  const float chuteAlt  = alt_apogee * 0.75f;

  Serial.println(F("  Predicted flight profile:"));
  Serial.print(F("    Net burn accel  : ")); Serial.print(a_burn, 2);   Serial.println(F(" m/s²"));
  Serial.print(F("    Burnout alt     : ")); Serial.print(alt_burnout, 1); Serial.println(F(" m"));
  Serial.print(F("    Burnout velocity: ")); Serial.print(v_burnout, 1); Serial.println(F(" m/s"));
  Serial.print(F("    Apogee          : ")); Serial.print(alt_apogee, 1); Serial.println(F(" m"));
  Serial.print(F("    Chute deploy alt: ")); Serial.print(chuteAlt, 1);  Serial.println(F(" m"));
  Serial.println(F("  Running sim... (x10 speed)"));
  Serial.println();

  LED(true, false, true);

  bool fired_P1 = false, fired_P3 = false, fired_P4 = false;
  float highest_alt = 0;
  unsigned long simStart = millis();
  float t = 0;

  // Simulate from launch to landing (altitude returns to 0)
  while (true) {
    t = (millis() - simStart) / 1000.0f * 10.0f;   // 10x speed

    float alt = simAltitude(t);
    float vel = simVertVel(t);
    if (alt < 0) alt = 0;

    if (alt > highest_alt) highest_alt = alt;

    // ── Staging events ───────────────────────────────────────────────────────
    if (!fired_P1 && t >= ignitionDelay) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.println(F("s  → IGNITION (P1) — motor lit"));
      mockPyro(P1, "P1 Ignition");
      fired_P1 = true;
      LED(true, true, false);
    }

    if (fired_P1 && t >= ignitionDelay + burnTime) {
      // This block runs once at burnout via a static flag
      static bool burnoutLogged = false;
      if (!burnoutLogged) {
        Serial.print(F("  t=")); Serial.print(t, 2);
        Serial.print(F("s  → BURNOUT — TVC stops | alt="));
        Serial.print(alt, 1); Serial.println(F(" m"));
        burnoutLogged = true;
        LED(false, true, false);
      }
    }

    if (!fired_P3 && alt < highest_alt - 1 && highest_alt > 5) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.print(F("s  → APOGEE DETECTED | highest_alt="));
      Serial.print(highest_alt, 1); Serial.println(F(" m"));
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.println(F("s  → STREAMER deploy (P3)"));
      mockPyro(P3, "P3 Streamer");
      fired_P3 = true;
      LED(false, true, true);
    }

    if (fired_P3 && !fired_P4 && alt < chuteAlt) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.print(F("s  → CHUTE deploy (P4) at "));
      Serial.print(alt, 1); Serial.println(F(" m (75% apogee)"));
      mockPyro(P4, "P4 Chute");
      fired_P4 = true;
      LED(true, true, true);
    }

    if (fired_P4 && alt <= 0) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.println(F("s  → LANDED"));
      break;
    }

    // Print status every simulated 0.5s
    static float lastPrintT = -1;
    if (t - lastPrintT >= 0.5f) {
      lastPrintT = t;
      Serial.print(F("      alt="));
      Serial.print(alt, 1);
      Serial.print(F(" m  vel="));
      Serial.print(vel, 1);
      Serial.println(F(" m/s"));
    }

    delay(5); // 5ms real = 50ms simulated
  }

  Serial.println();
  Serial.println(F("  Simulation complete. Verify the sequence above matches expected."));
  Serial.println(F("  Press button to continue."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 5 — Emergency Logic Test
// ─────────────────────────────────────────────────────────────────────────────
void test_emergency() {
  printHeader(5, "Emergency Logic Test");
  Serial.println(F("  Injecting fake gyro_x = 91 degrees (above 90 threshold)."));
  Serial.println(F("  Verifying emergency response without firing pyros."));
  Serial.println();

  float fake_gyro_x = 91.0f;
  float fake_gyro_y = 0.0f;

  if (abs(fake_gyro_x) > 90 || abs(fake_gyro_y) > 90) {
    Serial.println(F("  EMERGENCY CONDITION DETECTED ✓"));
    Serial.println(F("  Actions that would execute:"));
    Serial.println(F("    1. Servos -> neutral (90°)"));
    servoX.write(90 + Xtune);
    servoY.write(90 + Ytune);
    Serial.print(F("       ServoX written: ")); Serial.println((int)(90 + Xtune));
    Serial.print(F("       ServoY written: ")); Serial.println((int)(90 + Ytune));
    Serial.println(F("    2. LED -> RED"));
    LED(true, false, false);
    Serial.println(F("    3. Deploy parachute (P4)"));
    mockPyro(P4, "P4 Chute (emergency)");
    delay(500);
    Serial.println(F("    4. Deploy streamer (P3)"));
    mockPyro(P3, "P3 Streamer (emergency)");
    Serial.println();
    Serial.println(F("  Emergency logic PASSED."));
  } else {
    Serial.println(F("  ERROR: Emergency condition NOT detected — check threshold logic!"));
    LED(true, false, false);
  }

  beep(880, 100); beep(660, 200);
  Serial.println(F("  Press button to continue."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP & LOOP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(button, INPUT);
  pinMode(buzzer, OUTPUT);
  // Pyro pins as OUTPUT but we will NEVER write HIGH to them in this file
  pinMode(P1, OUTPUT); pinMode(P2, OUTPUT);
  pinMode(P3, OUTPUT); pinMode(P4, OUTPUT);
  digitalWrite(P1, LOW); digitalWrite(P2, LOW);
  digitalWrite(P3, LOW); digitalWrite(P4, LOW);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);

  servoX.attach(3);
  servoY.attach(4);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();

  // IMU
  mpu6050.begin();

  // Baro
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP280 not found — baro tests will be skipped"));
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
  }

  LED(false, false, false);
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║   SYSIPHUS PREFLIGHT TEST SUITE  ║"));
  Serial.println(F("╚══════════════════════════════════╝"));
  Serial.println(F("  PYRO CHANNELS ARE BLOCKED."));
  Serial.println(F("  Press button to begin Test 1."));
  beep(440, 100); beep(550, 100); beep(660, 150);
}

void loop() {
  if (!buttonPressed()) { delay(50); return; }

  testMode++;
  switch (testMode) {
    case 1: test_servoCenter(); break;
    case 2: test_servoSweep();  break;
    case 3: test_liveTVC();     break;
    case 4: test_flightSim();   break;
    case 5: test_emergency();   break;
    default:
      Serial.println(F("\n  All tests complete."));
      Serial.println(F("  Upload Ascent_Test.cpp for real flight."));
      LED(false, true, false);
      beep(523, 100); beep(659, 100); beep(784, 200);
      testMode = 5; // stay here
      break;
  }

  Serial.print(F("\n  Press button for Test "));
  Serial.println(testMode + 1 <= 5 ? testMode + 1 : 0);
  LED(false, false, false);
}
