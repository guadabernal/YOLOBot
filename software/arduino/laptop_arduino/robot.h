#ifndef ROBOT_H
#define ROBOT_H
#include <PID_v1.h>

// Pin definitions
#define PIN_DIR_RIGHT 5
#define PIN_SPEED_RIGHT 6
#define PIN_INPUT_RIGHT 21

#define PIN_DIR_LEFT 8
#define PIN_SPEED_LEFT 9
#define PIN_INPUT_LEFT 20

const float ticksPerRevolution = 90;				            // Probably I will need adjust this
const float robotDiameter = .468; 						        // [m]
const float wheelRadius = 0.0986;       							// [m]
const float wheelCircumference = 0.620; 							// [m] 2 * PI * wheel_radius
const float distanceCoef = wheelCircumference / ticksPerRevolution;
const float rotateCoef = 3.1415926 * robotDiameter / 360; 		// Angle in gradians
const float ticksPerMeter = ticksPerRevolution / wheelCircumference;

class Robot
{
public:
  enum Command { STOP, FORWARD, ROTATE };

  Robot()
  : leftSpeedPWM(0)
  , leftTicks(0)
  , leftTargetTicks(0)
  , leftTargetDirection(0)
  , rightSpeedPWM(0)
  , rightTicks(0)
  , rightTargetTicks(0)
  , rightTargetDirection(0)
  , leftPID(&leftTicks, &leftSpeedPWM, &leftTargetTicks, 10, 1, 2, P_ON_M, DIRECT)
  , rightPID(&rightTicks, &rightSpeedPWM, &rightTargetTicks, 10, 1, 2, P_ON_M, DIRECT)
  {
    leftPID.SetOutputLimits(0, 150);
    leftPID.SetSampleTime(20);
    rightPID.SetOutputLimits(0, 150);
    rightPID.SetSampleTime(20);
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
  }

  void executeCommand(int16_t command, int16_t val) {
  	switch (command) {
      case FORWARD:
        forward(val / 32767.0 * 100.0); // 0..32767 -- 0..100m
        break;
      case ROTATE:
        rotate(val / 32767.0 * 180.0);  // -180..180
        break;
      case STOP:
        stopNow();
        break;
    }
  }

  void rotate(float angle) {
  	if (angle > 180) angle = angle - 360;
    float distance = abs(angle) * rotateCoef;
    leftTargetTicks = distance * ticksPerMeter;
    rightTargetTicks = distance * ticksPerMeter;
    leftTargetDirection = angle > 0 ? 1 : -1;
    rightTargetDirection = angle > 0 ? -1 : 1;
    leftTicks = 0;
    rightTicks = 0;
    leftMinVel = 25;
    rightMinVel = 25;
    leftPID.SetTunings(10, 1, 2, P_ON_M);
    rightPID.SetTunings(10, 1, 2, P_ON_M);
    leftPID.SetOutputLimits(leftMinVel, 150);
    rightPID.SetOutputLimits(rightMinVel, 150);
    leftPID.Initialize();
    rightPID.Initialize();

//    Serial.print(leftTargetTicks);
//    Serial.print("-");
//    Serial.print(rightTargetTicks);
//    Serial.print("-");
//    Serial.print(distance);
//    Serial.print("-");
//    Serial.println(angle);
    stopped = false;
  }

  void forward(float distance) {
    leftTargetTicks = distance * ticksPerMeter;
    rightTargetTicks = distance * ticksPerMeter;
  	leftTargetDirection = 1;
  	rightTargetDirection = 1;
    leftTicks = 0;
    rightTicks = 0;
    leftPID.SetTunings(5, 1.5, 1, P_ON_M);
    rightPID.SetTunings(5, 1.5, 1, P_ON_M);
    leftMinVel = 0;
    rightMinVel = 0;
    leftPID.SetOutputLimits(leftMinVel, 100);
    rightPID.SetOutputLimits(rightMinVel, 100);
    leftPID.Initialize();
    rightPID.Initialize();
    
    stopped = false;
  }

  bool stopped = true;
  void stopNow() {
  	leftSpeed(0, 0);
  	rightSpeed(0, 0);
    stopped = true;
  }

  void updateState(int lTicks, int rTicks) {
    if (stopped) return;
    leftTicks = lTicks;
    rightTicks = rTicks;
    leftPID.Compute();
    rightPID.Compute();
    if (leftSpeedPWM >0 || rightSpeedPWM > 0) {
      Serial.print(leftTicks);
      Serial.print(", ");
      Serial.print(leftTargetTicks);
      Serial.print(", ");
      Serial.print(rightTicks);
      Serial.print(", ");
      Serial.print(rightTargetTicks);
      Serial.print("    -  ");
      Serial.print(leftSpeedPWM);      
      Serial.print(", ");
      Serial.println(rightSpeedPWM);      
    }
        
//    float errTicks = abs(rightTicks - leftTicks);
//    if (errTicks > 1) {
//      if (rightTicks > leftTicks) leftSpeedPWM += 5;
//      else rightSpeedPWM += 5;
//      if (leftSpeedPWM > 150) leftSpeedPWM = 150;
//      if (rightSpeedPWM > 150) rightSpeedPWM = 150;
//    }
     if(leftSpeedPWM <= leftMinVel) leftSpeedPWM = 0;
     if(rightSpeedPWM <= rightMinVel) rightSpeedPWM = 0;
     leftSpeed(leftTargetDirection, leftSpeedPWM);
     rightSpeed(rightTargetDirection, rightSpeedPWM);
  }

protected:
  void leftSpeed(int direction, float speed) {
    digitalWrite(PIN_DIR_LEFT, (direction >= 0) ? HIGH : LOW);
    analogWrite(PIN_SPEED_LEFT, speed);
  }

  void rightSpeed(int direction, float speed) {
    digitalWrite(PIN_DIR_RIGHT, (direction >= 0) ? LOW : HIGH);
    analogWrite(PIN_SPEED_RIGHT, speed);
  }
private:
  double leftSpeedPWM;
  double leftTicks;
  double leftTargetTicks;
  int leftTargetDirection;
  double rightSpeedPWM;
  double rightTicks;
  double rightTargetTicks;
  int rightTargetDirection;
  float leftMinVel;
  float rightMinVel;
  
  PID leftPID;
  PID rightPID;
};

#endif
