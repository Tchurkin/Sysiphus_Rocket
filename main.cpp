#include <SPI.h>                 // Include SPI library for SD communication
#include <SD.h>                  // Include SD library for SD card operations
#include <Wire.h>                // Include Wire library for I2C communication
#include <MPU6050_tockn.h>       // Include MPU6050 library for IMU (Inertial Measurement Unit)
#include <Adafruit_BMP280.h>     // Include BMP280 library for barometric pressure sensor
#include <PWMServo.h>            // Include PWMServo library for controlling servos

// Pin definitions
constexpr int button = 9;                 // Button for input
constexpr int buzzer = 10;                // Buzzer for audio
constexpr int P1 = 4;                     // Ignition for primary stage
constexpr int P2 = 5;                     // Ignition for secondary stage
constexpr int P3 = 6;                     // Melt wire for landing leg deployment
constexpr int P4 = 11;                    // Melt wire for parachute ejection
constexpr int LED = 13;                   // Onboard LED

// Servo adjustments
const float Xtune = 0, Ytune = 0;    // Tune values for servo neutral positions

Adafruit_BMP280 bmp;             // BMP280 barometric sensor object
MPU6050 mpu6050(Wire);           // MPU6050 sensor object using I2C

PWMServo servoX, servoY; // Servo objects for thrust vector control (TVC)

// Constants for TVC control and flight dynamics
const double ServoXMult = 7, ServoYMult = 7;  // Multipliers for servo angles
const double P = 0.3, D = 0.2;               // PID controller constants (Proportional and Derivative)
const double burnTime = 3.45;                // Rocket motor burn time (seconds)
const double avThrust = 14.34;              // Average thrust (Newtons)
const double rocketWeight = 1.2;            // Rocket weight (kilograms)
const double G = 9.74;                      // Gravitational acceleration, experimental (m/s^2)
const double deltaAlt = (((avThrust / rocketWeight) - G) * (burnTime * burnTime)) / 2; // Predicted altitude gain during burn

// Variables for flight data
float gyro_x, gyro_y, ang_vel_x, ang_vel_y, accel_x, accel_y, accel_z, gyro_z;
float temp, altitude, initial_alt, highest_alt, vert_vel;
float servo_x, servo_y, servo_tiltX, servo_tiltY;

void emergency();
void sensors();
void beep(int frequency, int duration);
bool buttonCount();
void countdown();
void TVC();
void descent();

void setup() {
  // Set pin modes
  pinMode(button, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);
  pinMode(P3, OUTPUT);
  pinMode(P4, OUTPUT);
  pinMode(LED, OUTPUT);

  // Initialize servos
  servoX.attach(2);
  servoY.attach(3);

  // Start Serial for debugging
  Serial.begin(9600);

  // Initialize all outputs to LOW
  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(P3, LOW);
  digitalWrite(P4, LOW);

  // Initialize I2C communication
  Wire.begin();

  // Initialize BMP280 sensor
  bmp.begin(0x76);  // Address of BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_125);

  // Initialize MPU6050 sensor
  mpu6050.begin();

}

void emergency() {
  // Check for extreme tilts and trigger emergency mode if needed
  if (abs(gyro_x) > 90 || abs(gyro_y) > 90) {
    Serial.print(F("EMERGENCY"));
    digitalWrite(P4, HIGH);
    delay(1000);
    digitalWrite(P3, HIGH);
    while (1) {
      // Flash LED and beep repeatedly
      digitalWrite(LED, HIGH);
      beep(500, 50);
      digitalWrite(LED, LOW);
      delay(50);
      if (digitalRead(button) == HIGH) {
        while (true) {
          digitalWrite(LED, HIGH);
          delay(250);
          digitalWrite(LED, LOW);
          delay(250);
        }
      }
    }
  }
}

void sensors() {
  // Update sensor data and calculate velocity
  static long prevTime = millis();
  static float prev_alt;

  mpu6050.update();  // Update MPU6050 sensor data
  altitude = bmp.readAltitude(1013.25) - initial_alt;  // Calculate current altitude
  temp = mpu6050.getTemp();  // Get temperature from MPU6050
  gyro_x = mpu6050.getAngleX();
  gyro_y = mpu6050.getAngleY();
  gyro_z = mpu6050.getAngleZ();
  ang_vel_x = mpu6050.getGyroX();
  ang_vel_y = mpu6050.getGyroY();
  accel_x = mpu6050.getAccX() * G;
  accel_y = mpu6050.getAccY() * G;
  accel_z = mpu6050.getAccZ() * G;

  
  long currentTime = millis(); // time in seconds
  long elapsedTime = currentTime - prevTime; // time difference
  
  if (elapsedTime >= 10) {
    
    // Baro altitude deriving to get vertical velocity
    vert_vel = (altitude - prev_alt) / elapsedTime * 1000;

    // reset values for deriving
    prev_alt = altitude;
    prevTime = currentTime;
  }

  if (altitude > highest_alt) {
    highest_alt = altitude;
  }

  // Check for emergency conditions
  emergency();
  Serial.println(altitude);
}


void beep(int frequency, int duration) {
  // Generate a tone on the buzzer
  tone(buzzer, frequency);
  delay(duration);
  noTone(buzzer);
}

bool buttonCount() {
  // Detect button presses within a time window
  const int pressWindow = 300;  // Time window for counting presses
  static unsigned long lastPressTime = 0;
  static int pressCount = 0;

  bool buttonState = digitalRead(button);

  if (buttonState == HIGH) {
    if (millis() - lastPressTime <= pressWindow) {
      pressCount++;
      digitalWrite(LED, HIGH);
      tone(buzzer, 311.13 * pow(2, float(pressCount) / 12.0));  // Generate ascending tone
      while (digitalRead(button) == HIGH) {}  // Wait for button release
      noTone(buzzer);
      digitalWrite(LED, LOW);
    } else {
      pressCount = 1;  // Reset press count
    }
    lastPressTime = millis();
  }

  return (pressCount > 5);  // Return true if the button is pressed more than 5 times
}

void countdown() {
  // Countdown sequence before launch
  const unsigned long countdownDuration = 10;

  servoX.write(90 + Xtune);
  servoY.write(90 + Ytune);
  
  for (int i = countdownDuration; i > 0; i--) {
    if (i > 5) {
      Serial.println(i);
      digitalWrite(LED, HIGH);
      beep(440, 200);  // Beep for each second
      digitalWrite(LED, LOW);
      delay(800);
    } else {
      Serial.println(i);
      digitalWrite(LED, HIGH);
      tone(buzzer, 440);  // Continuous tone for the last 5 seconds
      delay(1000);
    }
  }
  // CALIBRATION
  tone(buzzer, 880); // Higher pitched buzz to show calibration
  mpu6050.calcGyroOffsets(true, 0, 0);  // Perform gyro offset calibration
  initial_alt = bmp.readAltitude(1013.25);  // Read initial altitude
  Serial.println(initial_alt);

  // LAUNCH
  Serial.println(F("LAUNCH"));  // Indicate launch
  noTone(buzzer);
  digitalWrite(LED, LOW);
  digitalWrite(P1, HIGH);  // Ignite primary stage
}

void TVC() {
  // Thrust Vector Control (TVC) logic
  sensors();  // Update sensor data
  float motor_tiltX = (P * gyro_x) + (D * ang_vel_x);
  servo_tiltX = constrain(motor_tiltX, -5, 5) * ServoXMult;  // Calculate tilt for X-axis
  servo_tiltX = servo_tiltX + 90 + Xtune;
  servoX.write(servo_tiltX);

  float motor_tiltY = (P * gyro_y) + (D * ang_vel_y);
  servo_tiltY = constrain(motor_tiltY, -5, 5) * ServoYMult;  // Calculate tilt for Y-axis
  servo_tiltY = servo_tiltY + 90 + Ytune;
  servoY.write(servo_tiltY);
}

void descent() {
  // Logic for descent phase
  servoX.write(90 + Xtune);  // Reset servos to neutral
  servoY.write(90 + Ytune);

  while (altitude > deltaAlt) {
    sensors();  // Update sensor data
  }

  digitalWrite(P2, HIGH);  // Ignite secondary stage
  digitalWrite(LED, HIGH);

  while (altitude > 0.5 || accel_z > 10 || vert_vel < -0.5) {
    TVC();  // Perform TVC during descent
    if (altitude < 10) {
      digitalWrite(P3, HIGH);  // Deploy landing legs closer to ground
    }
  }
  servoX.write(90 + Xtune);  // Reset servos to neutral
  servoY.write(90 + Ytune);
}

void loop() {
  // Main loop
  while (!buttonCount()) {
    delay(50);  // Wait for button sequence to initiate
  }
  delay(500);
  countdown();  // Start countdown sequence
  while (altitude > highest_alt - 1) {
    TVC();  // Perform TVC during ascent
  }
  beep(659, 100);
  beep(523, 100);
  beep(659, 100);
  descent();  // Perform descent logic
  digitalWrite(LED, LOW);
  Serial.print(F("LANDED"));
  while (digitalRead(button) == LOW) {
    digitalWrite(LED, HIGH);
    beep(523, 1000);
    digitalWrite(LED, LOW);
    beep(392, 1000);
  }
  while (true) {}  // End program
}
