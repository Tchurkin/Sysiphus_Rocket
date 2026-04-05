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
    6. Sensor Readout     — live stream of all IMU and baro values

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
constexpr int P1     = 9;    // Melt wire for streamer deployment
constexpr int P2     = 10;   // Not connected
constexpr int P3     = 11;   // Ignition for ascent motor
constexpr int P4     = 12;   // Melt wire for parachute ejection
constexpr int RLED   = 6;
constexpr int GLED   = 7;
constexpr int BLED   = 8;

// ── Tuning (keep in sync with Ascent_Test.cpp) ──────────────────────────────
const float  Xtune       = 0,    Ytune      = 0;
const double ServoXMult  = 4,    ServoYMult = 4;
const double P_GAIN      = 0.1, D_GAIN     = 0.1;
const double burnTime    = 3.45;
const double avThrust    = 14.34;
const double rocketWeight= 0.78;
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

// Mirrors the countdown() sequence in Ascent_Test.cpp exactly.
// Call before any test that needs calibrated gyro + initial altitude.
float sim_initial_alt = 0;
void runCountdown(int duration) {
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
  for (int i = duration; i > 0; i--) {
    if (i > 5) {
      Serial.println(i);
      LED(true, false, false);
      beep(440, 200);
      LED(false, false, false);
      delay(800);
    } else if (i > 3) {
      Serial.println(i);
      LED(true, false, false);
      tone(buzzer, 440);
      delay(1000);
    } else if (i == 3) {
      Serial.println(i);
      LED(true, false, true);           // purple = calibrating
      tone(buzzer, 880);
      mpu6050.calcGyroOffsets(true, 0, 0);
      sim_initial_alt = bmp.readAltitude(1013.25);
      Serial.print(F("  initial_alt: ")); Serial.println(sim_initial_alt);
      noTone(buzzer);
      LED(false, false, false);
    }
    // i=2, i=1 fall through instantly
  }
  Serial.println(F("  [LAUNCH]"));
}

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
  float raw_x =   mpu6050.getAngleX() + Xtune;
  float raw_y = -(mpu6050.getAngleY() + Ytune);
  float raw_z =   mpu6050.getAngleZ();
  float raw_av_x =  mpu6050.getGyroX();
  float raw_av_y = -mpu6050.getGyroY();

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
  int nx = (int)(90 + Xtune), ny = (int)(90 + Ytune);
  Serial.print(F("  Writing ServoX -> ")); Serial.println(nx);
  Serial.print(F("  Writing ServoY -> ")); Serial.println(ny);
  servoX.write(nx);
  servoY.write(ny);
  LED(false, true, false);
  Serial.println(F("  Verify both TVC fins are physically centred, then press button."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 2 — Servo Sweep
// ─────────────────────────────────────────────────────────────────────────────
void test_servoSweep() {
  printHeader(2, "Servo Sweep");
  Serial.println(F("  Sweeping -5 -> +5 -> -5 on each axis. Watch the gimbal move."));
  Serial.println(F("  Format:  axis  tilt(deg)  servo_cmd(deg)"));
  LED(false, false, true);

  // Sweep X positive
  Serial.println(F("\n  [X] sweeping negative -> positive"));
  for (int deg = -5; deg <= 5; deg++) {
    int cmd = (int)(deg * ServoXMult + 90 + Xtune);
    servoX.write(cmd);
    Serial.print(F("  X  ")); Serial.print(deg > 0 ? "+" : "");
    Serial.print(deg); Serial.print(F("°  ->  srv ")); Serial.println(cmd);
    delay(200);
  }
  Serial.println(F("  [X] returning to neutral"));
  for (int deg = 5; deg >= -5; deg--) {
    servoX.write((int)(deg * ServoXMult + 90 + Xtune));
    delay(200);
  }
  servoX.write(90 + Xtune);
  Serial.print(F("  [X] neutral: srv ")); Serial.println((int)(90 + Xtune));

  delay(500);

  // Sweep Y
  Serial.println(F("\n  [Y] sweeping negative -> positive"));
  for (int deg = -5; deg <= 5; deg++) {
    int cmd = (int)(deg * ServoYMult + 90 + Ytune);
    servoY.write(cmd);
    Serial.print(F("  Y  ")); Serial.print(deg > 0 ? "+" : "");
    Serial.print(deg); Serial.print(F("°  ->  srv ")); Serial.println(cmd);
    delay(200);
  }
  Serial.println(F("  [Y] returning to neutral"));
  for (int deg = 5; deg >= -5; deg--) {
    servoY.write((int)(deg * ServoYMult + 90 + Ytune));
    delay(200);
  }
  servoY.write(90 + Ytune);
  Serial.print(F("  [Y] neutral: srv ")); Serial.println((int)(90 + Ytune));

  Serial.println(F("\n  Sweep complete. Press button to continue."));
  while (!buttonPressed()) delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 3 — Live TVC
// ─────────────────────────────────────────────────────────────────────────────
void test_liveTVC() {
  printHeader(3, "Live TVC");
  Serial.println(F("  Running countdown — keep rocket still and upright."));
  runCountdown(5);
  Serial.println(F("  Tilt the rocket slowly and watch servo response."));
  Serial.println(F("  EXPECTED: servo moves OPPOSITE to lean (correcting)."));
  Serial.println(F("  Press button to exit."));
  Serial.println();
  Serial.println(F("  GyroX   GyroY   SrvX_cmd  SrvY_cmd  Diagnosis"));

  LED(false, true, true);
  unsigned long lastPrint = 0;
  // Track previous angles to detect which axis is actively moving
  float prev_gx = 0, prev_gy = 0;
  unsigned long prevAxisTime = millis();

  Serial.println(F("  Angle(°)        AngVel(°/s)     ServoCmd        Status"));
  Serial.println(F("  X       Y       X       Y       X       Y"));

  while (!buttonPressed()) {
    readIMU();
    computeTVC();
    servoX.write(tiltX + 90 + Xtune);
    servoY.write(tiltY + 90 + Ytune);

    if (millis() - lastPrint > 150) {
      unsigned long now = millis();
      float dt = (now - prevAxisTime) / 1000.0f;
      prevAxisTime = now;
      lastPrint = now;

      // Rate of angle change over the print interval
      float dGx = abs(gyro_x - prev_gx) / dt;
      float dGy = abs(gyro_y - prev_gy) / dt;
      prev_gx = gyro_x; prev_gy = gyro_y;

      // Dominant axis: whichever is moving faster
      bool xDominant = dGx > dGy && dGx > 2.0f;
      bool yDominant = dGy > dGx && dGy > 2.0f;

      // Servo at correction limit?
      bool xLimited = abs(tiltX) >= 4.9f * ServoXMult;
      bool yLimited = abs(tiltY) >= 4.9f * ServoYMult;

      // Sign check: tilt command should share sign with gyro angle
      // (positive lean -> positive servo deflection to correct)
      bool xSignOk = abs(gyro_x) < 1.0f || (gyro_x > 0) == (tiltX > 0);
      bool ySignOk = abs(gyro_y) < 1.0f || (gyro_y > 0) == (tiltY > 0);

      // Angles
      Serial.print(F("  "));
      Serial.print(gyro_x >= 0 ? "+" : ""); Serial.print(gyro_x, 1);
      Serial.print(F("  "));
      Serial.print(gyro_y >= 0 ? "+" : ""); Serial.print(gyro_y, 1);
      // Angular velocities
      Serial.print(F("  "));
      Serial.print(ang_vel_x >= 0 ? "+" : ""); Serial.print(ang_vel_x, 1);
      Serial.print(F("  "));
      Serial.print(ang_vel_y >= 0 ? "+" : ""); Serial.print(ang_vel_y, 1);
      // Servo commands (degrees from neutral)
      Serial.print(F("  "));
      Serial.print(tiltX >= 0 ? "+" : ""); Serial.print(tiltX, 1);
      Serial.print(F("  "));
      Serial.print(tiltY >= 0 ? "+" : ""); Serial.print(tiltY, 1);
      // Status
      Serial.print(F("  "));
      if (xDominant)        Serial.print(F("[MOVING X]  "));
      else if (yDominant)   Serial.print(F("[MOVING Y]  "));
      else                  Serial.print(F("[level]     "));
      if (xLimited)         Serial.print(F("[X AT LIMIT]"));
      if (yLimited)         Serial.print(F("[Y AT LIMIT]"));
      if (!xSignOk)         Serial.print(F(" <<WARN: X sign may be wrong>>"));
      if (!ySignOk)         Serial.print(F(" <<WARN: Y sign may be wrong>>"));
      Serial.println();
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
  Serial.println(F("  Predicted flight profile:"));
  Serial.print(F("    Net burn accel  : ")); Serial.print(a_burn, 2);   Serial.println(F(" m/s²"));
  Serial.print(F("    Burnout alt     : ")); Serial.print(alt_burnout, 1); Serial.println(F(" m"));
  Serial.print(F("    Burnout velocity: ")); Serial.print(v_burnout, 1); Serial.println(F(" m/s"));
  Serial.print(F("    Apogee          : ")); Serial.print(alt_apogee, 1); Serial.println(F(" m"));
  Serial.print(F("    Chute deploy alt: ")); Serial.print(alt_apogee - 1, 1); Serial.println(F(" m  (apogee - 1 m)"));
  Serial.println(F("  Running countdown then sim... (sim at x10 speed)"));
  Serial.println();
  runCountdown(5);
  Serial.println();

  LED(true, false, true);

  bool fired_ignition = false, fired_apogee = false;
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
    if (!fired_ignition && t >= ignitionDelay) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.println(F("s  → IGNITION (P3) — motor lit"));
      mockPyro(P3, "P3 Ignition");
      fired_ignition = true;
      LED(true, true, false);
    }

    if (fired_ignition && t >= ignitionDelay + burnTime) {
      static bool burnoutLogged = false;
      if (!burnoutLogged) {
        Serial.print(F("  t=")); Serial.print(t, 2);
        Serial.print(F("s  → BURNOUT — TVC stops | alt="));
        Serial.print(alt, 1); Serial.println(F(" m"));
        burnoutLogged = true;
        LED(false, true, false);
      }
    }

    // Apogee detected + chute fires immediately (matches firmware: alt < highest_alt - 1)
    if (!fired_apogee && alt < highest_alt - 1 && highest_alt > 5) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.print(F("s  → APOGEE DETECTED | highest_alt="));
      Serial.print(highest_alt, 1); Serial.println(F(" m"));
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.print(F("s  → CHUTE deploy (P4) at "));
      Serial.print(alt, 1); Serial.println(F(" m  (apogee - 1 m)"));
      mockPyro(P4, "P4 Chute");
      fired_apogee = true;
      LED(false, true, true);
    }

    if (fired_apogee && alt <= 0) {
      Serial.print(F("  t=")); Serial.print(t, 2);
      Serial.println(F("s  → LANDED"));
      break;
    }

    // Print status every simulated 0.5s
    static float lastPrintT = -1;
    if (t - lastPrintT >= 0.5f) {
      lastPrintT = t;
      const char* phase =
        t < ignitionDelay              ? "pre-ignition " :
        t < ignitionDelay + burnTime   ? "POWERED (TVC)" :
        vel > 0                        ? "coast up     " :
        !fired_apogee                  ? "coast down   " :
                                         "chute descent";
      Serial.print(F("      ["));
      Serial.print(phase);
      Serial.print(F("]  alt="));
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
  Serial.println(F("  Injecting fake gyro_x = 46 degrees (above 45 threshold)."));
  Serial.println(F("  Verifying emergency response without firing pyros."));
  Serial.println();

  float fake_gyro_x = 46.0f;
  float fake_gyro_y = 0.0f;

  if (abs(fake_gyro_x) > 45 || abs(fake_gyro_y) > 45) {
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
    Serial.println(F("    4. Deploy streamer (P1)"));
    mockPyro(P1, "P1 Streamer (emergency)");
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
//  TEST 6 — Sensor Readout
// ─────────────────────────────────────────────────────────────────────────────
void test_sensorReadout() {
  printHeader(6, "Sensor Readout");
  Serial.println(F("  Live sensor values at 5 Hz. Tilt, move, and verify responses."));
  Serial.println(F("  Press button to exit.\n"));

  LED(false, true, false);
  unsigned long lastPrint = 0;
  int rowCount = 0;

  while (!buttonPressed()) {
    mpu6050.update();

    if (millis() - lastPrint < 200) { delay(5); continue; }
    lastPrint = millis();

    // Re-print column headers every 15 rows so they stay visible while scrolling
    if (rowCount % 15 == 0) {
      Serial.println(F("  AngleX  AngleY  AngleZ |  AvX    AvY  |  AccX   AccY   AccZ  | Alt(m)  Temp(C)"));
      Serial.println(F("  ------  ------  ------ | ----   ---- |  ----   ----   ----  | ------  -------"));
    }
    rowCount++;

    float aX  = mpu6050.getAngleX();
    float aY  = mpu6050.getAngleY();
    float aZ  = mpu6050.getAngleZ();
    float avX = mpu6050.getGyroX();
    float avY = mpu6050.getGyroY();
    float acX = mpu6050.getAccX() * 9.81f;
    float acY = mpu6050.getAccY() * 9.81f;
    float acZ = mpu6050.getAccZ() * 9.81f;
    float alt = bmp.readAltitude(1013.25);
    float tmp = bmp.readTemperature();

    char buf[96];
    snprintf(buf, sizeof(buf),
      "  %+6.1f  %+6.1f  %+6.1f | %+5.1f  %+5.1f | %+6.2f %+6.2f %+6.2f | %+6.1f   %4.1f",
      aX, aY, aZ, avX, avY, acX, acY, acZ, alt, tmp);
    Serial.println(buf);
  }

  Serial.println(F("\n  Press button to continue."));
  delay(200);
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
  digitalWrite(RLED, HIGH); digitalWrite(GLED, HIGH); digitalWrite(BLED, HIGH); // all LEDs off

  servoX.attach(3);
  servoY.attach(4);

  Serial.begin(115200);
  // Blink blue — open Serial Monitor, then press button to begin.
  // (Teensy's 'while (!Serial)' is not reliable — button press is.)
  while (digitalRead(button) == LOW) {
    digitalWrite(BLED, LOW);  delay(200);
    digitalWrite(BLED, HIGH); delay(200);
  }
  waitButtonRelease();
  delay(300);

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
  Serial.println(F("  PYRO CHANNELS ARE BLOCKED.\n"));

  // Sensor health check
  Serial.println(F("  -- Sensor Check --"));
  mpu6050.update();
  Serial.print(F("  MPU6050 temp  : ")); Serial.print(mpu6050.getTemp(), 1); Serial.println(F(" C"));
  Serial.print(F("  BMP280 alt    : ")); Serial.print(bmp.readAltitude(1013.25), 1); Serial.println(F(" m (raw, no baseline)"));
  Serial.print(F("  BMP280 temp   : ")); Serial.print(bmp.readTemperature(), 1); Serial.println(F(" C"));
  Serial.print(F("  ServoX neutral: ")); Serial.println((int)(90 + Xtune));
  Serial.print(F("  ServoY neutral: ")); Serial.println((int)(90 + Ytune));
  Serial.println();
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
    case 5: test_emergency();     break;
    case 6: test_sensorReadout(); break;
    default:
      Serial.println(F("\n  All tests complete."));
      Serial.println(F("  Upload Ascent_Test.cpp for real flight."));
      LED(false, true, false);
      beep(523, 100); beep(659, 100); beep(784, 200);
      testMode = 6; // stay here
      break;
  }

  Serial.print(F("\n  Press button for Test "));
  Serial.println(testMode + 1 <= 6 ? testMode + 1 : 0);
  LED(false, false, false);
}
