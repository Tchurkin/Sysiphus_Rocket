#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <PWMServo.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

// Pin definitions
constexpr int button = 14;                 // Button for input
constexpr int buzzer = 13;                // Buzzer for audio
constexpr int P1 = 9;                     // Ignition for primary stage
constexpr int P2 = 10;                     // Ignition for secondary stage
constexpr int P3 = 11;                     // Melt wire for streamer deployment
constexpr int P4 = 12;                    // Melt wire for parachute ejection
constexpr int RLED = 6;
constexpr int GLED = 7;
constexpr int BLED = 8;
const int chipSelect = BUILTIN_SDCARD;

// --- Servo and Gyro Calibration ---
// NOTE: axis signs may need flipping depending on MPU6050 mounting orientation
const float Xtune = 1, Ytune = 0;
PWMServo servoX, servoY; // Servo objects for thrust vector control (TVC)

File logFile;  // SD log file object

// --- IMU and Baro ---
MPU6050 mpu6050(Wire);
Adafruit_BMP280 bmp;

// --- Constants ---
const double ServoXMult = 4, ServoYMult = 4;
const double P = 0.4, D = 0.3;
const double burnTime = 3.45; // seconds
const double avThrust = 14.34; // N
const double rocketWeight = 0.7; // kg
const double G = 9.81; // ms^2
const double ignitionDelay = 0.2; // seconds
const double delayDrop = 0.5 * G * ignitionDelay * ignitionDelay;
const double deltaAlt = 0.5 * ((avThrust / rocketWeight) - G) * burnTime * burnTime - delayDrop; // delta alt

// --- Flight Variables ---
float temp, altitude, initial_alt, highest_alt, vert_vel;
float raw_gyro_x, raw_gyro_y, raw_gyro_z, raw_ang_vel_x, raw_ang_vel_y; // raw
float gyro_x, gyro_y, gyro_z, ang_vel_x, ang_vel_y; // filtered
float accel_x, accel_y, accel_z;
float servo_tiltX, servo_tiltY;
float tiltX, tiltY;

// --- Pyro Control ---
struct PyroChannel {
  int pin;
  bool active;
  unsigned long startTime;
};

PyroChannel pyros[] = {
  {P1, false, 0},
  {P2, false, 0},
  {P3, false, 0},
  {P4, false, 0}
};

void triggerPyro(int pin) {
  for (int i = 0; i < 4; i++) {
    if (pyros[i].pin == pin && !pyros[i].active) {
      digitalWrite(pin, HIGH);
      pyros[i].active = true;
      pyros[i].startTime = millis();
    }
  }
}

void updatePyros() {
  for (int i = 0; i < 4; i++) {
    if (pyros[i].active && millis() - pyros[i].startTime >= 1000) {
      digitalWrite(pyros[i].pin, LOW);
      pyros[i].active = false;
    }
  }
}


void setup() {
  pinMode(button, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(P1, OUTPUT); pinMode(P2, OUTPUT); pinMode(P3, OUTPUT); pinMode(P4, OUTPUT);
  pinMode(RLED, OUTPUT); pinMode(GLED, OUTPUT); pinMode(BLED, OUTPUT);

  digitalWrite(P1,LOW);
  digitalWrite(P2,LOW);
  digitalWrite(P3,LOW);
  digitalWrite(P4,LOW);

  servoX.attach(3);
  servoY.attach(4);

  LED(false, false, false);

  Serial.begin(115200);

  Wire.begin();

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);

  mpu6050.begin();

  // Initialize SD card
  SD.begin(chipSelect);
}

void sensors() {
  static long prevTime = millis();
  static float prev_alt;

  mpu6050.update();

  // MPU6050_tockn gives angles in degrees (integrated gyro) and gyro in deg/s
  // Map axes to match rocket orientation — swap/negate if TVC corrects wrong way
  raw_gyro_x = mpu6050.getAngleX() + Xtune;
  raw_gyro_y = mpu6050.getAngleY() + Ytune;
  raw_gyro_z = mpu6050.getAngleZ();
  raw_ang_vel_x = mpu6050.getGyroX();   // deg/s, already — no *180/PI needed
  raw_ang_vel_y = mpu6050.getGyroY();

  accel_x = mpu6050.getAccX() * G;
  accel_y = mpu6050.getAccY() * G;
  accel_z = mpu6050.getAccZ() * G;

  temp = mpu6050.getTemp();

  altitude = bmp.readAltitude(1013.25) - initial_alt;

  float gyro_alpha = 0.9;
  float ang_vel_alpha = 0.9;
  // Smoothing Filter
  gyro_x = gyro_alpha * raw_gyro_x + (1 - gyro_alpha) * gyro_x;
  gyro_y = gyro_alpha * raw_gyro_y + (1 - gyro_alpha) * gyro_y;
  gyro_z = gyro_alpha * raw_gyro_z + (1 - gyro_alpha) * gyro_z;
  ang_vel_x = ang_vel_alpha * raw_ang_vel_x + (1 - ang_vel_alpha) * ang_vel_x;
  ang_vel_y = ang_vel_alpha * raw_ang_vel_y + (1 - ang_vel_alpha) * ang_vel_y;

  long currentTime = millis();
  long elapsedTime = currentTime - prevTime;
  if (elapsedTime >= 50) {
    float raw_vert_vel = (altitude - prev_alt) / (elapsedTime / 1000.0);
    vert_vel = 0.2 * raw_vert_vel + 0.8 * vert_vel;
    prev_alt = altitude;
    prevTime = currentTime;

    logData();
  }

  if (altitude > highest_alt) highest_alt = altitude;
  updatePyros();
  emergency();
}

char filename[20];

void createUniqueLogFile() {
  int fileIndex = 0;
  do {
    sprintf(filename, "LOG%03d.CSV", fileIndex++);
  } while (SD.exists(filename) && fileIndex < 1000);

  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    Serial.print("Logging to: ");
    Serial.println(filename);
    logFile.println("Time(ms),Altitude(m),VertVel(m/s),GyroX,GyroY,GyroZ,AngVelX,AngVelY,AccelX,AccelY,AccelZ,ServoX,ServoY");
    logFile.close();
  } else {
    Serial.println("Failed to create log file!");
  }
}

void logData() {
  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    // Format CSV line with relevant data
    logFile.print(millis());
    logFile.print(",");
    logFile.print(altitude);
    logFile.print(",");
    logFile.print(vert_vel);
    logFile.print(",");
    logFile.print(gyro_x);
    logFile.print(",");
    logFile.print(gyro_y);
    logFile.print(",");
    logFile.print(gyro_z);
    logFile.print(",");
    logFile.print(ang_vel_x);
    logFile.print(",");
    logFile.print(ang_vel_y);
    logFile.print(",");
    logFile.print(accel_x);
    logFile.print(",");
    logFile.print(accel_y);
    logFile.print(",");
    logFile.print(accel_z);
    logFile.print(",");
    logFile.print(constrain(tiltX, -5, 5));
    logFile.print(",");
    logFile.println(constrain(tiltY, -5, 5));
    logFile.close();
  }
}

void emergency() {
  if (abs(gyro_x) > 90 || abs(gyro_y) > 90) {
    servoX.write(90 + Xtune);
    servoY.write(90 + Ytune);
    Serial.println("EMERGENCY");
    LED(true,false,false);
    triggerPyro(P4);  // deploy parachute
    delay(1000);
    triggerPyro(P3);  // deploy streamer
    while (1) {
      updatePyros();
      beep(500, 50);
      delay(50);
      if (digitalRead(button) == HIGH) while (1) updatePyros();
    }
  }
}

void beep(int frequency, int duration) {
  tone(buzzer, frequency);
  delay(duration);
  noTone(buzzer);
}

void LED(bool red, bool green, bool blue) {
  digitalWrite(RLED, !red);
  digitalWrite(GLED, !green);
  digitalWrite(BLED, !blue);
}

bool buttonCount() {
  const int pressWindow = 300;
  static unsigned long lastPressTime = 0;
  static int pressCount = 0;

  bool buttonState = digitalRead(button);

  if (buttonState == HIGH) {
    if (millis() - lastPressTime <= pressWindow) {
      pressCount++;
      LED(false,true,false);
      tone(buzzer, 311.13 * pow(2, float(pressCount) / 12.0));
      while (digitalRead(button) == HIGH) {}
      noTone(buzzer);
      LED(false,false,false);
    } else {
      pressCount = 1;
    }
    lastPressTime = millis();
  }

  return (pressCount > 5);
}

void countdown() {
  const unsigned long countdownDuration = 30;
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);

  createUniqueLogFile();

  for (int i = countdownDuration; i > 0; i--) {
    if (i > 5) {
      Serial.println(i);
      LED(true,false,false);
      beep(440, 200);
      LED(false,false,false);
      delay(800);
    } else if (i > 1) {
      Serial.println(i);
      LED(true,false,false);
      tone(buzzer, 440);
      delay(1000);
    } else {
      Serial.println(i);
      LED(true,false,true);
      // Calibrate before buzzer — vibration from tone corrupts gyro samples
      mpu6050.calcGyroOffsets(true, 0, 0);
      initial_alt = bmp.readAltitude(1013.25);
      Serial.println(initial_alt);
      tone(buzzer, 880);
    }
  }

  Serial.println(F("LAUNCH"));
  triggerPyro(P1);
}

void TVC() {
  sensors();
  tiltX = constrain(P * gyro_x + D * ang_vel_x, -5, 5) * ServoXMult;
  tiltY = constrain(P * gyro_y + D * ang_vel_y, -5, 5) * ServoYMult;
  servoX.write(tiltX + 90 + Xtune);
  servoY.write(tiltY + 90 + Ytune);
}

void descent() {
  triggerPyro(P3);
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
  while (altitude > deltaAlt) sensors();
  triggerPyro(P2);
  LED(true,false,true);
  long start = millis();
  while (millis() - start < 3000) TVC();
  triggerPyro(P4);
  while (altitude > 0.5 || accel_z > 10 || vert_vel < -0.5) TVC();
  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
}

void loop() {
  LED(true,true,true);
  delay(500);
  LED(false,false,false);
  while (!buttonCount()) delay(50);
  delay(500);
  countdown();
  unsigned long launchTime = millis();
  LED(true,true,false);
  while (altitude > highest_alt - 1) {
    if (millis() - launchTime < (burnTime + ignitionDelay) * 1000) {
      TVC();
    } else {
      sensors();
      servoX.write(90 + Xtune);
      servoY.write(90 + Ytune);
    }
  }
  beep(659, 100); beep(523, 100); beep(659, 100);
  LED(false,true,true);
  triggerPyro(P3);  // deploy streamer at apogee

  // deploy parachute at 75% of apogee altitude on the way down
  float chuteAlt = highest_alt * 0.75;
  while (altitude > chuteAlt) sensors();
  triggerPyro(P4);

  // descent();
  LED(true,true,true);
  Serial.println("LANDED");
  while (digitalRead(button) == LOW) {
    beep(523, 1000);
    beep(392, 1000);
    updatePyros();
  }
  LED(false,false,false);
  while (true) updatePyros();
}
