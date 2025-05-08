// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define SD_CHIP_SELECT 10 // SD card pin
#define SERVO_PIN 9       // Servo Pin

// Flight states
enum FlightState { WAITING, IDLE, LIFTOFF, BURNOUT, APOGEE, TOUCHDOWN };
FlightState state = WAITING;

// LED flashing variables
bool LEDflash = false;        // Define if LED flashes or not
const long LEDinterval = 500; // how fast LED flashes
unsigned long LEDTimer = 0;
bool ledState = false;        // define the state (on or off) the LED is at.

// Other flags to prevent repeated actions.
static bool datalogging = false;
static bool servoRotated = false;

// Variables to keep track of flight time.
unsigned long flightTime = 0;
unsigned long liftoffTime = 0;
const long autoThreshhold = 6500; // define how long until servo auto rotates.

Servo servo;    // name the call to servo
File dataFile;  // name the call to sd card file

// define acceleration & gyro in x, y, z:
float accelX, accelY, accelZ;   // Acceleration in X, Y, Z axes
float gyroX, gyroY, gyroZ;      // Gyroscope in X, Y, Z axes
float angleY = 0, angleZ = 0;   // define the total angle from gyro to 0
float totalAccel;               // Variable to store total acceleration in 'g'
unsigned long imuTimer;         // define previous time to calculate total gyro displacement. 

// sets LED call
void setLED(bool r, bool g, bool b) {
  digitalWrite(LEDR, r);
  digitalWrite(LEDG, g);
  digitalWrite(LEDB, b);
}

// sets servo rotate call
void rotateServo() {
  servo.write(180);
  delay(1000);
  servo.write(90);
}

// turns on LED and slightly rotates servo at beginning of oporation.
void setup() {
  Serial.begin(9600);
  setLED(HIGH, HIGH, LOW);  // blue = IDLE
  servo.attach(SERVO_PIN);
  delay(2000);

  Serial.println("Quick Servo Testing");
  delay(1000);
  servo.write(180);
  delay(100);
  servo.write(90);
  delay(100);
  servo.write(0);
  delay(100);
  servo.write(90);
  delay(1000);
  Serial.println("Quick Servo Test Complete");

  begin();
}

void begin() {
  Serial.println("Type 'S' to begin, 'T' to test servo motor: ");
  while (true) {
    if (Serial.available()) {
      char command = Serial.read();
      if (command == 'S') {
        state = IDLE;
        setLED(HIGH, HIGH, LOW);  // blue
        Serial.println("Starting...");
        break;
      } else if (command == 'T') {
        setLED(HIGH, LOW, HIGH);  // green
        Serial.println("Testing Servo...");
        rotateServo();
        Serial.println("Servo Test Complete!");
      }
    }
  }
}

// all flight oporations take place in the loop function. 
void loop() {
  if (state == IDLE) {
    imuTimer = millis();
    LEDflash = true;

    // initialize IMU
    if (!IMU.begin()) {
      Serial.println("IMU initialization failed!");
      state = WAITING;
      begin();
    } else {
      Serial.println("IMU initialized!");
    }

    // initialize SD card
    if (!SD.begin(SD_CHIP_SELECT)) {
      Serial.println("SD card initialization failed!");
      state = WAITING;
      begin();
    } else {
      Serial.println("SD module initialized!");
    }

    // initialize data logging
    dataFile = SD.open("IMU_DATA.txt", FILE_WRITE);
    if (dataFile) {
      Serial.println("File Opened!");
      dataFile.println("Time, AccelX, Total Accel, Gyro Y, Gyro Z");
    } else {
      Serial.println("Error opening file");
      state = WAITING;
      begin();
    }
  }

  while (state != WAITING) {
    flightTime = millis() - liftoffTime;

    // auto rotate servo if it has not already after a set interval.
    if (!servoRotated && state != IDLE && flightTime >= autoThreshhold) {
      rotateServo();
      Serial.println("Auto Rotating Servo!");
      dataFile.println("Auto Rotating Servo!");
      servoRotated = true;
    }

    // begin data logging
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(accelX, accelY, accelZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);

      // Calculate time difference (deltaTime)
      unsigned long currentTime = millis();
      float deltaTime = (currentTime - imuTimer) / 1000.0;
      imuTimer = currentTime;

      // Integrate angular velocity (gyroscope) to estimate total angle (in degrees)
      angleY += gyroY * deltaTime;
      angleZ += gyroZ * deltaTime;

      // Calculate total acceleration in 'g'
      totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

      if (dataFile) {
        static bool dataloggingprint = false;
        dataFile.print(flightTime);
        dataFile.print(", ");
        dataFile.print(accelX);
        dataFile.print(", ");
        dataFile.print(totalAccel);
        dataFile.print("°, ");
        dataFile.print(angleY);
        dataFile.print("°, ");
        dataFile.println(angleZ);

        Serial.print(flightTime);
        Serial.print(", ");
        Serial.print(accelX);
        Serial.print(", ");
        Serial.print(totalAccel);
        Serial.print(", ");
        Serial.print(angleY);
        Serial.print("°, ");
        Serial.println(angleZ);

        dataFile.flush(); // ensure data is logged in real-time, not just at the end.

        if (!dataloggingprint) {
          dataloggingprint = true;
          Serial.println("Beginning Data logging!");
        }
      } else {
        static bool errorPrinted = false;
        if (!errorPrinted) {
          Serial.println("Error writing the data to SD card!");
          errorPrinted = true;
          state = WAITING;
        }
      }
    }

    // Detect liftoff
    if (state == IDLE && totalAccel >= 1.75) {
      state = LIFTOFF;
      liftoffTime = millis();
      Serial.println("Liftoff Detected!");
      dataFile.println("Liftoff Detected!");
    }

    // Detect burnout
    if (state == LIFTOFF && totalAccel <= 0.45) {
      state = BURNOUT;
      Serial.println("Burnout Detected!");
      dataFile.println("Burnout Detected!");
      angleY = 0;
      angleZ = 0;
    }

    // Detect apogee
    if (state == BURNOUT && !servoRotated &&
        (totalAccel >= 0.95 || abs(angleZ) >= 65 || abs(angleY) >= 65)) {
      state = APOGEE;
      Serial.println("Apogee Detected!");
      dataFile.println("Apogee Detected!");
    }

    // rotate servo at apogee
    if (state == APOGEE && !servoRotated) {
      rotateServo();
      Serial.println("Rotating Servo!");
      dataFile.println("Rotating Servo!");
      servoRotated = true;
    }

    // detect touchdown
    if (state == APOGEE && totalAccel >= 1.5) {
      state = TOUCHDOWN;
      Serial.println("Touchdown Detected!");
      dataFile.println("Touchdown Detected!");
    }

    // stop programme after 10 seconds if touchdown occured. 
    if (state == TOUCHDOWN) {
      LEDflash = false;
      delay(10000);
      Serial.println("Ending programme!");
      dataFile.println("Ending programme!");
      setLED(HIGH, HIGH, HIGH);  // all off
      dataFile.close();
      while (1);
    }

    // flash LED during flight.
    if (LEDflash) {
      unsigned long currentMillis = millis();
      if (currentMillis - LEDTimer >= LEDinterval) {
        LEDTimer = currentMillis;
        ledState = !ledState;
        digitalWrite(LEDB, ledState ? LOW : HIGH);
      }
    }
  }
}