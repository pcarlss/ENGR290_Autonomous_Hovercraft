#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

//straight-angle
const int straightAngle = 96;
//Fans
bool start = true;
const int echoPin = 3;    // port 13 for US
const int trigPin = 13;   // port 13
const int thrustFan = 6;  // port 4
const int liftFan = 5;

//Servo
Servo servo;
int angle = 0;
const int servoPin = 9;  // port 9

//MPU6050
MPU6050 mpu(Wire);
int yaw = 0;
float initialYaw = 0;

//----------------------------------

void setup() {
  Serial.begin(115200);

  //imu
  Wire.begin();
  //Fans
  pinMode(liftFan, OUTPUT);
  pinMode(thrustFan, OUTPUT);

  //Servo
  servo.attach(servoPin);

  //US Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //MPU6050
  byte status = mpu.begin();
  Serial.print(F("\n\nMPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));

  mpu.calcOffsets();

  servo.write(straightAngle);  //reset servo straight
  delay(1000);
  analogWrite(thrustFan, 255);
  analogWrite(liftFan, 255);
}

//----------------------------------

void makeDecision(float distance_cm) {
  int counter = 0;

  //update IMU
  mpu.update();
  yaw = mpu.getAngleZ();

  //check distance
  if (distance_cm <= 40) {
    Serial.println("[Slowing Down]");
    analogWrite(thrustFan, 0);
    analogWrite(liftFan, 0);
    delay(2000);

    //bump into wall
    Serial.println("[Hugging Wall]");
    analogWrite(thrustFan, 255);
    analogWrite(liftFan, 255);
    
    while (counter < 150) {
      //MPU6050 yaw drift adjustment
      mpu.update();
      yaw = mpu.getAngleZ();

      Serial.print("Current yaw: ");
      Serial.println(yaw);

      //make servo adjustment based on filtered yaw
      if (yaw > (initialYaw + 0.5)) {
        Serial.print(straightAngle + (yaw - initialYaw));
        servo.write(straightAngle + (yaw - initialYaw));
      } else if (yaw < (initialYaw - 0.5)) {
        Serial.print(straightAngle + (yaw - initialYaw));
        servo.write(straightAngle + (yaw - initialYaw));
      } else {
        servo.write(straightAngle);
      }

      delay(10);
      counter++;
    }

    //stop in place
    analogWrite(thrustFan, 0);
    analogWrite(liftFan, 0);
    delay(500);
    angle = functionSweep();

    if (angle == straightAngle) {
      servo.write(straightAngle);

      delay(500);
      analogWrite(liftFan, 255);
      analogWrite(thrustFan, 255);
    } else {
      //turn accordingly
      returnToCenter(angle);

      analogWrite(liftFan, 255);
      analogWrite(thrustFan, 255);
    }
  } else {
    goStraight(initialYaw);
  }
}  //----------------------------------

void goStraight(float initialYaw) {

  Serial.print("[GO STRAIGHT] ");

  analogWrite(thrustFan, 255);
  analogWrite(liftFan, 255);

  //MPU6050 yaw drift adjustment
  mpu.update();
  yaw = mpu.getAngleZ();

  Serial.print("Current yaw: ");
  Serial.print(yaw);

  //make servo adjustment based on filtered yaw
  if (yaw > (initialYaw + 0.5)) {
    Serial.print(straightAngle + (yaw - initialYaw));
    servo.write(straightAngle + (yaw - initialYaw));
  } else if (yaw < (initialYaw - 0.5)) {
    Serial.print(straightAngle + (yaw - initialYaw));
    servo.write(straightAngle + (yaw - initialYaw));
  } else {
    servo.write(straightAngle);
  }
}


//----------------------------------

int functionSweep() {
  Serial.println("[Sweeping]");
  int left = servoRead(0);
  int right = servoRead(180);


  if ((right <= 50 && left < 30) || (right < 30 && left <= 50)) {
    return straightAngle;
  } else if (right > left) {
    servo.write(180);
    delay(500);
    return 180;
  } else {
    servo.write(0);
    delay(500);
    return 0;
  }
}

//----------------------------------

int servoRead(int angle) {
  servo.write(angle);
  delay(1000);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration / 58.0;

  Serial.print("\nAngle: ");
  Serial.print(angle);
  Serial.print(", Distance: ");
  Serial.print(distance_cm);
  Serial.print(" cm");
  delay(500);
  return distance_cm;
}

//----------------------------------

void returnToCenter(int angle) {
  if (angle > 90) {
    int initialServoAngle = 180;
    int targetYaw = yaw - 90;
    int currentYaw = yaw;
    int error = targetYaw - currentYaw;

    Serial.print(" target yaw: ");
    Serial.print(targetYaw);
    Serial.print(" current yaw: ");
    Serial.print(yaw);
    Serial.print(" error: ");
    Serial.println(error);

    analogWrite(liftFan, 245);
    analogWrite(thrustFan, 220);

    while (currentYaw >= targetYaw + 50) {
      Serial.print("[RIGHT] ");
      Serial.print("error: ");
      Serial.print(error);

      mpu.update();
      currentYaw = mpu.getAngleZ();
      error = targetYaw - currentYaw;

      //seervo angle based on error
      int servoAngle = initialServoAngle - (90 + error);
      servo.write(servoAngle);

      //thrustSpeed based on error
      analogWrite(thrustFan, 255 + error);

      Serial.print(" InitialServoAngle: ");
      Serial.print(initialServoAngle);

      Serial.print("ServoAngle: ");
      Serial.print(servoAngle);

      Serial.print("targetYaw: ");
      Serial.print(targetYaw);

      Serial.print("currentYaw: ");
      Serial.println(currentYaw);

      delay(20);
    }
  } else {
    int initialServoAngle = 0;
    int targetYaw = yaw + 90;
    int currentYaw = yaw;
    int error = targetYaw - currentYaw;

    Serial.print(" target yaw: ");
    Serial.print(targetYaw);
    Serial.print(" current yaw: ");
    Serial.print(yaw);
    Serial.print(" error: ");
    Serial.println(error);

    analogWrite(liftFan, 245);
    analogWrite(thrustFan, 220);

    while (currentYaw <= targetYaw - 50) {
      Serial.print("[LEFT] ");
      Serial.print("error: ");
      Serial.print(error);

      mpu.update();
      currentYaw = mpu.getAngleZ();
      error = targetYaw - currentYaw;

      //servo angle base on error
      int servoAngle = initialServoAngle + (90 - error);
      servo.write(servoAngle);

      //thrustSpeed based on error
      analogWrite(thrustFan, 255 - error);

      Serial.print("InitialServoAngle: ");
      Serial.print(initialServoAngle);

      Serial.print(" ServoAngle: ");
      Serial.print(servoAngle);

      Serial.print(" targetYaw: ");
      Serial.print(targetYaw);

      Serial.print(" currentYaw: ");
      Serial.println(currentYaw);

      delay(20);
    }
  }
  //Let turn complete
  analogWrite(liftFan, 0);
  analogWrite(thrustFan, 0);
  Serial.print("DELAY YAW RESET");
  delay(2000);
  Serial.print("READING YAW");

  mpu.update();
  initialYaw = mpu.getAngleZ();
  // initialYaw = snapToDeg(mpu.getAngleZ());

  //correct yaw to go straight
  // goStraight(initialYaw);
}

// int snapToDeg(float yaw) {
//   int snappedYaw = yaw;
//   if (yaw > -45 && yaw <= 45) {
//     snappedYaw = 0;
//   } else if (yaw > 45 && yaw <= 135) {
//     snappedYaw = 90;
//   } else if (yaw > 135 && yaw <= -135) {
//     snappedYaw = 180;
//   } else if (yaw > -135 && yaw <= -45) {
//     snappedYaw = -90;
//   } else {
//   };
//   return snappedYaw;
// }

//-----------------------------------//
// LOOP LOOP LOOP LOOP //

void loop() {

  //US sensor check distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration / 58.0;
  Serial.print(" Current Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm ");

  //make decision based on distance
  makeDecision(distance_cm);
}
