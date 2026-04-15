/*
  Sensor_Test.cpp — Interactive sensor output viewer
  ---------------------------------------------------
  Runs the 30-second drift calibration at startup (same logic as the
  research flight), then streams one sensor group at a time to serial.
  Press button to advance to the next group.

  Verifying calibration:
    After the 30s countdown, phase 3 (ANGULAR RATE) should read ~0 dps
    on all axes when the rocket is still. If it reads more than ~1 dps,
    calibration did not work. Phase 2 (GYRO ANGLES) should stay near 0°
    over time when still — noticeable drift means bad calibration.

  Recalibrate: hold the button at startup, or press the reset button.

  Test order:
    1. Tilt angle      — accel-based deg from vertical (what TVC actually uses)
    2. Gyro angles     — pure gyro integration X/Y/Z (deg)
    3. Angular rate    — calibrated gyro X/Y/Z (deg/s)
    4. Acceleration    — X/Y/Z in g and m/s²
    5. Altitude        — BMP280 altitude (m) and temperature (°C)
*/

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
constexpr int BUTTON = 14;
constexpr int BUZZER = 13;
constexpr int RLED   = 6;
constexpr int GLED   = 7;
constexpr int BLED   = 8;

// ── Constants ─────────────────────────────────────────────────────────────────
constexpr float G             = 9.81f;
constexpr float SEA_LEVEL_HPA = 1013.25f;
constexpr int   CAL_DURATION  = 30;   // seconds — same as research flight

// ── Hardware ──────────────────────────────────────────────────────────────────
MPU6050         mpu6050(Wire);
Adafruit_BMP280 bmp;

// ── Pure gyro angle reference (for phase 2 display) ───────────────────────────
// Library auto-integrates getGyroAngleX/Y/Z on every update(); we just
// snapshot a reference when entering the phase and show the delta.
float refGyroX = 0, refGyroY = 0, refGyroZ = 0;

// ── Utilities ─────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r); digitalWrite(GLED, !g); digitalWrite(BLED, !b);
}

void beep(int freq, int dur) { tone(BUZZER, freq); delay(dur); noTone(BUZZER); }

// ── 30-second drift calibration (mirrors Research_Flight.cpp) ─────────────────
// Rocket must be still on the pad. Integrated raw gyro angle over the
// countdown IS the accumulated drift; average rate = bias offset.
void calibrateGyro() {
  Serial.println(F("\n── GYRO CALIBRATION ──"));
  Serial.print(F("Keep still for ")); Serial.print(CAL_DURATION); Serial.println(F("s"));

  // Read initial gyro-integrated angle (library auto-accumulates on update())
  mpu6050.update();
  float start_x = mpu6050.getGyroAngleX();
  float start_y = mpu6050.getGyroAngleY();
  float start_z = mpu6050.getGyroAngleZ();
  unsigned long t0 = millis();

  for (int i = CAL_DURATION; i > 0; i--) {
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
  float drift_z = mpu6050.getGyroAngleZ() - start_z;
  float offX = drift_x / elapsed;
  float offY = drift_y / elapsed;
  float offZ = drift_z / elapsed;
  mpu6050.setGyroOffsets(offX, offY, offZ);

  Serial.println(F("\n── RESULTS ──"));
  Serial.print(F("Elapsed:           ")); Serial.print(elapsed, 2); Serial.println(F(" s"));
  Serial.print(F("Drift angle (deg): X=")); Serial.print(drift_x, 2);
  Serial.print(F("  Y=")); Serial.print(drift_y, 2);
  Serial.print(F("  Z=")); Serial.println(drift_z, 2);
  Serial.print(F("Offset (dps):      X=")); Serial.print(offX, 3);
  Serial.print(F("  Y=")); Serial.print(offY, 3);
  Serial.print(F("  Z=")); Serial.println(offZ, 3);

  // Verify: average 50 samples of post-calibration rate — should be ~0
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 50; i++) {
    mpu6050.update();
    sx += mpu6050.getGyroX();
    sy += mpu6050.getGyroY();
    sz += mpu6050.getGyroZ();
    delay(10);
  }
  Serial.print(F("Post-cal rate avg: X=")); Serial.print(sx / 50, 3);
  Serial.print(F("  Y=")); Serial.print(sy / 50, 3);
  Serial.print(F("  Z=")); Serial.println(sz / 50, 3);
  Serial.println(F("(Post-cal rates should be near 0 dps)\n"));

  beep(523, 80); beep(659, 80); beep(784, 120);
}

// ── Test phases ───────────────────────────────────────────────────────────────
constexpr int NUM_TESTS = 5;
int testPhase = 0;

const bool PHASE_LED[NUM_TESTS][3] = {
  {false, false, true },   // 1 tilt angle    — BLUE
  {false, true,  false},   // 2 gyro angles   — GREEN
  {true,  true,  false},   // 3 angular rate  — YELLOW
  {true,  false, false},   // 4 acceleration  — RED
  {true,  false, true },   // 5 altitude      — PURPLE
};

const char* PHASE_HEADER[NUM_TESTS] = {
  "\n── 1. TILT ANGLE (accel-based, deg from vertical) ──",
  "\n── 2. GYRO ANGLES (pure integration — should stay near 0 if still) ──",
  "\n── 3. ANGULAR RATE (deg/s — should read ~0 if still and calibrated) ──",
  "\n── 4. ACCELERATION (g  |  m/s²) ──",
  "\n── 5. ALTITUDE & TEMPERATURE ──",
};

void enterPhase(int p) {
  LED(PHASE_LED[p][0], PHASE_LED[p][1], PHASE_LED[p][2]);
  Serial.println(PHASE_HEADER[p]);
  if (p == 1) {   // snapshot library gyro angle so phase shows delta from entry
    mpu6050.update();
    refGyroX = mpu6050.getGyroAngleX();
    refGyroY = mpu6050.getGyroAngleY();
    refGyroZ = mpu6050.getGyroAngleZ();
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(BUTTON, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);
  digitalWrite(RLED, HIGH); digitalWrite(GLED, HIGH); digitalWrite(BLED, HIGH);

  Serial.begin(115200);
  Wire.begin();

  if (!bmp.begin(0x76)) { Serial.println(F("BMP280 not found")); while (true); }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);

  mpu6050.begin();
  Wire.setClock(400000);   // 400 kHz I2C after init — higher sample rate

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║      SYSIPHUS SENSOR TEST        ║"));
  Serial.println(F("╚══════════════════════════════════╝"));

  calibrateGyro();

  Serial.println(F("Press button to advance to next sensor group."));
  enterPhase(0);
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop() {
  mpu6050.update();   // sample every iteration so fast motion is captured

  // Throttle serial output to 5 Hz — library keeps integrating at full rate
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) {
    // Still check button during the idle window
    if (digitalRead(BUTTON) == HIGH) {
      delay(30);
      while (digitalRead(BUTTON) == HIGH) delay(10);
      testPhase = (testPhase + 1) % NUM_TESTS;
      beep(880, 120);
      enterPhase(testPhase);
    }
    return;
  }
  lastPrint = millis();

  switch (testPhase) {

    case 0: {   // Tilt angle — accel-based, 0° = vertical
      float gx   = mpu6050.getAccX();
      float gy   = mpu6050.getAccY();
      float gz   = mpu6050.getAccZ();
      float tilt = atan2f(sqrtf(gx*gx + gy*gy), gz) * 180.0f / M_PI;
      Serial.print(F("Tilt: ")); Serial.print(tilt, 2); Serial.println(F(" deg"));
      break;
    }

    case 1: {   // Pure gyro angles — library auto-integrated, relative to entry
      Serial.print(F("GyroAngX: "));   Serial.print(  mpu6050.getGyroAngleX() - refGyroX, 2);
      Serial.print(F("  GyroAngY: ")); Serial.print(-(mpu6050.getGyroAngleY() - refGyroY), 2);   // Y inverted
      Serial.print(F("  GyroAngZ: ")); Serial.println(mpu6050.getGyroAngleZ() - refGyroZ, 2);
      break;
    }

    case 2: {   // Angular rate — calibrated gyro
      Serial.print(F("GyroX: "));   Serial.print(  mpu6050.getGyroX(), 2);
      Serial.print(F("  GyroY: ")); Serial.print( -mpu6050.getGyroY(), 2);   // Y inverted
      Serial.print(F("  GyroZ: ")); Serial.println(mpu6050.getGyroZ(), 2);
      break;
    }

    case 3: {   // Acceleration
      float ax = mpu6050.getAccX(), ay = -mpu6050.getAccY(), az = mpu6050.getAccZ();   // Y inverted
      Serial.print(F("X: ")); Serial.print(ax, 3); Serial.print(F("g / "));
      Serial.print(ax * G, 2); Serial.print(F(" m/s²    "));
      Serial.print(F("Y: ")); Serial.print(ay, 3); Serial.print(F("g / "));
      Serial.print(ay * G, 2); Serial.print(F(" m/s²    "));
      Serial.print(F("Z: ")); Serial.print(az, 3); Serial.print(F("g / "));
      Serial.print(az * G, 2); Serial.println(F(" m/s²"));
      break;
    }

    case 4: {   // Altitude and temperature
      Serial.print(F("Altitude: ")); Serial.print(bmp.readAltitude(SEA_LEVEL_HPA), 2);
      Serial.print(F(" m    Temp: ")); Serial.print(bmp.readTemperature(), 1);
      Serial.println(F(" °C"));
      break;
    }
  }

  if (digitalRead(BUTTON) == HIGH) {
    delay(30);
    while (digitalRead(BUTTON) == HIGH) delay(10);
    testPhase = (testPhase + 1) % NUM_TESTS;
    beep(880, 120);
    enterPhase(testPhase);
  }
}
