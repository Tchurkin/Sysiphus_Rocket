/*
  Sensor_Test.cpp — Interactive sensor output viewer
  ---------------------------------------------------
  Streams one sensor group at a time to serial.
  Press button to advance to the next group.

  Test order:
    1. Tilt angle      — accel-based deg from vertical (what TVC actually uses)
    2. Gyro angles     — complementary filter X/Y/Z (deg)
    3. Angular rate    — raw gyro X/Y/Z (deg/s)
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

// ── Hardware ──────────────────────────────────────────────────────────────────
MPU6050         mpu6050(Wire);
Adafruit_BMP280 bmp;

// ── Utilities ─────────────────────────────────────────────────────────────────
void LED(bool r, bool g, bool b) {
  digitalWrite(RLED, !r); digitalWrite(GLED, !g); digitalWrite(BLED, !b);
}

void beep(int freq, int dur) { tone(BUZZER, freq); delay(dur); noTone(BUZZER); }

// ── Test phases ───────────────────────────────────────────────────────────────
constexpr int NUM_TESTS = 5;
int testPhase = 0;

// LED color and header for each phase
const bool PHASE_LED[NUM_TESTS][3] = {
  {false, false, true },   // 1 tilt angle    — BLUE
  {false, true,  false},   // 2 gyro angles   — GREEN
  {true,  true,  false},   // 3 angular rate  — YELLOW
  {true,  false, false},   // 4 acceleration  — RED
  {true,  false, true },   // 5 altitude      — PURPLE
};

const char* PHASE_HEADER[NUM_TESTS] = {
  "\n── 1. TILT ANGLE (accel-based, deg from vertical) ──",
  "\n── 2. GYRO ANGLES (complementary filter, deg) ──",
  "\n── 3. ANGULAR RATE (deg/s) ──",
  "\n── 4. ACCELERATION (g  |  m/s²) ──",
  "\n── 5. ALTITUDE & TEMPERATURE ──",
};

void enterPhase(int p) {
  LED(PHASE_LED[p][0], PHASE_LED[p][1], PHASE_LED[p][2]);
  Serial.println(PHASE_HEADER[p]);
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

  // Quick gyro calibration — keep rocket still
  Serial.println(F("Calibrating gyro (keep still)..."));
  LED(true, false, true);
  mpu6050.calcGyroOffsets(true, 0, 0);
  LED(false, false, false);

  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║      SYSIPHUS SENSOR TEST        ║"));
  Serial.println(F("╚══════════════════════════════════╝"));
  Serial.println(F("Press button to advance to next sensor group."));

  beep(523, 80); beep(659, 80); beep(784, 120);

  enterPhase(0);
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
void loop() {
  mpu6050.update();

  // ── Print current sensor group ──────────────────────────────────────────────
  switch (testPhase) {

    case 0: {   // Tilt angle — accel-based, 0° = vertical
      float gx   = mpu6050.getAccX();
      float gy   = mpu6050.getAccY();
      float gz   = mpu6050.getAccZ();
      float tilt = atan2f(sqrtf(gx*gx + gy*gy), gz) * 180.0f / M_PI;
      Serial.print(F("Tilt: ")); Serial.print(tilt, 2); Serial.println(F(" deg"));
      break;
    }

    case 1: {   // Gyro angles — complementary filter
      Serial.print(F("AngleX: ")); Serial.print(mpu6050.getAngleX(), 2);
      Serial.print(F("  AngleY: ")); Serial.print(mpu6050.getAngleY(), 2);
      Serial.print(F("  AngleZ: ")); Serial.println(mpu6050.getAngleZ(), 2);
      break;
    }

    case 2: {   // Angular rate — raw gyro
      Serial.print(F("GyroX: ")); Serial.print(mpu6050.getGyroX(), 2);
      Serial.print(F("  GyroY: ")); Serial.print(mpu6050.getGyroY(), 2);
      Serial.print(F("  GyroZ: ")); Serial.println(mpu6050.getGyroZ(), 2);
      break;
    }

    case 3: {   // Acceleration
      float ax = mpu6050.getAccX(), ay = mpu6050.getAccY(), az = mpu6050.getAccZ();
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

  // ── Button press → advance to next group ────────────────────────────────────
  if (digitalRead(BUTTON) == HIGH) {
    delay(30);
    while (digitalRead(BUTTON) == HIGH) delay(10);
    testPhase = (testPhase + 1) % NUM_TESTS;
    beep(880, 120);
    enterPhase(testPhase);
  }

  delay(200);
}
