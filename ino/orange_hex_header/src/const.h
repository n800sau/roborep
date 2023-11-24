#ifndef __CONST_H

#define __CONST_H

const int SONAR_TILT_INCR = 30;

const int SONAR_PAN_ANGLE_MIN = 35; // right side
const int SONAR_PAN_ANGLE_MAX = 135; // left side
const int SONAR_PAN_CENTER = SONAR_PAN_ANGLE_MIN + (SONAR_PAN_ANGLE_MAX - SONAR_PAN_ANGLE_MIN) / 2;

const int SONAR_TILT_ANGLE_MIN = 80; // top
const int SONAR_TILT_ANGLE_MAX = 145; // bottom
const int SONAR_TILT_CENTER = SONAR_TILT_ANGLE_MIN + (SONAR_TILT_ANGLE_MAX - SONAR_TILT_ANGLE_MIN) / 2;

// up-down
const int headTiltServoPin = 6;
// left-right
const int headPanServoPin = 5;

const int LED_PIN = 8;

const int INTERRUPT_PIN2 = 2;
const int INTERRUPT_PIN3 = 3;

#endif // __CONST_H
