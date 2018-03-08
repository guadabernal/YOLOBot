#ifndef ROBOT_H
#define ROBOT_H

//#include <PID_v1.h>

const float ticksPerRevolution = 90;				            // Probably I will need adjust this
const float robotDiameter = 456.9; 						        // [mm]
const float wheelRadius = 98.6;       							// [mm]
const float wheelCircumference = 620; 							// [mm] 2 * PI * wheel_radius
const float distanceCoef = wheel_circ / ticks_per_revolution;
const float rotateCoef = 3.1415926 * robotDiameter / 360; 		// Angle in gradians 


class Robot
{
public:
  enum Command { STOP, FORWARD, ROTATE };

  Robot()
  : speedLeftPWM(0)
  , speedRightPWM(0)
  , leftDistance(0)
  , rightDistance(0)
  , leftTargetDistance(0)
  , rightTargetDistance(0)
  {}

  void executeCommand(int16_t command, int16_t val) {
  	switch (command) {
      case FORWARD:
        forward(val); // 0..32768 -- 0..10m
        break;
      case ROTATE:
        rotate(val);  // -180..180
        break;
      case STOP:

        break;
    }
  }

  void rotate(float angle) {
  	if (angle > 180) angle = angle - 360;

  	leftTargetDistance = abs(angle) * rotateCoef;
  	rightTargetDistance = abs(angle) * rotateCoef;
  	leftTargetDirection = angle > 0 ? 1 : -1;
  	rightTargetDirection = angle > 0 ? -1 : 1;
  }

  void forward(float distance) {
  	leftTargetDistance = distance;
  	rightTargetDistance = distance;
  	leftTargetDirection = 1;
  	rightTargetDirection = 1;
  }

  void stop() {
  	rightTargetDistance = 0;
  	leftTargetDistance = 0;
  	leftSpeed(0);
  	rightSpeed(0);
  }

  void update(int lTicks, int rTicks) {
  	calculateDistance(lTicks, rTicks);
  	//setSpeeds();
  }

protected:
  void calculateDistance(int lTicks, int rTicks) {
    leftDistance = lTicks * distanceCoef;
    rightDistance = rTicks * distanceCoef;
  }

  void leftSpeed(int direction, float speed) {
    digitalWrite(PIN_DIR_LEFT, direction >= 0 ? HIGH : LOW);
    analogWrite(PIN_SPEED_LEFT, speed);
    speedLeftPWM = speed;
  }

  void rightSpeed(int direction, float speed) {
    digitalWrite(PIN_DIR_RIGHT, direction >= 0 ? LOW : HIGH);
    analogWrite(PIN_SPEED_RIGHT, speed);
    speedRightPWM = speed;
  }
private:
  float speedLeftPWM;
  float speedRightPWM;
  float leftDistance;
  float rightDistance;
  float leftTargetDistance;
  float rightTargetDistance;
  float leftDirection;
  float rightDirection;
  float leftTargetDirection;
  float rightTargetDirection;
};

#endif