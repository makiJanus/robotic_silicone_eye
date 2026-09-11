#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// 12 servos total: 6 per eye
#define NUM_SERVOS 12
#define EYE_SIZE 6
#define SERVO_FREQ 50

// Servo pulse limits (MG995)
#define SERVO_MIN_US 600
#define SERVO_MAX_US 2400

// Physical PCA9685 channel mapping
// Right eye: channels 0-5
// Left eye:  channels 8-13
// (channels 6, 7, 14, 15 unused)
const uint8_t SERVO_CHANNELS[NUM_SERVOS] = {
  0, 1, 2, 3, 4, 5,      // Right eye
  8, 9, 10, 11, 12, 13   // Left eye
};

// Per-eye servo order (same for both sides):
//   0: Eye Pan        (horizontal)
//   1: Eye Tilt       (vertical)
//   2: Upper Eyelid
//   3: Lower Eyelid
//   4: Eyebrow Inner
//   5: Eyebrow Outer

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pca9685.begin();
  pca9685.setPWMFreq(SERVO_FREQ);
  delay(100);

  // Initialize all servos to neutral
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 90);
    delay(20);
  }

  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("M")) {
      // Format: M<servo_id>,<angle>  (servo_id 0-11)
      int commaIndex = command.indexOf(',');
      if (commaIndex != -1) {
        int servoId = command.substring(1, commaIndex).toInt();
        int angle = command.substring(commaIndex + 1).toInt();

        if (servoId >= 0 && servoId < NUM_SERVOS) {
          setServoAngle(servoId, angle);
          Serial.println("OK");
        } else {
          Serial.println("ERROR: Invalid servo ID");
        }
      }
    }
    else if (command.startsWith("P,")) {
      // Format: P,a0,a1,...,a11  (12 values, in servo index order)
      int startIndex = 2;
      int servoCount = 0;
      while (startIndex < (int)command.length() && servoCount < NUM_SERVOS) {
        int commaIndex = command.indexOf(',', startIndex);
        int endIndex = (commaIndex == -1) ? command.length() : commaIndex;
        int angle = command.substring(startIndex, endIndex).toInt();
        setServoAngle(servoCount, angle);
        servoCount++;
        startIndex = endIndex + 1;
      }
      Serial.println("OK");
    }
    else if (command == "CENTER") {
      for (int i = 0; i < NUM_SERVOS; i++) {
        setServoAngle(i, 90);
        delay(20);
      }
      Serial.println("OK");
    }
    else if (command == "STATUS") {
      Serial.println("OK");
    }
    else if (command == "EYE_CALIBRATE") {
      calibrateEye();
      Serial.println("OK");
    }
  }
}

void setServoAngle(int servoId, int angle) {
  if (servoId < 0 || servoId >= NUM_SERVOS) return;
  angle = constrain(angle, 0, 180);

  float pulse = mapFloat(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  int ticks = (int)((pulse * 4096.0) / 20000.0 + 0.5);
  ticks = constrain(ticks, 0, 4095);

  // Map servo index -> physical PCA channel
  uint8_t channel = SERVO_CHANNELS[servoId];
  pca9685.setPWM(channel, 0, ticks);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calibrateEye() {
  // Sweep every servo through its range
  for (int i = 0; i < NUM_SERVOS; i++) {
    for (int angle = 0; angle <= 180; angle += 10) {
      setServoAngle(i, angle);
      delay(50);
    }
    setServoAngle(i, 90);
    delay(200);
  }
}