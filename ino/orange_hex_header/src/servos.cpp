#include "servos.h"
#include "const.h"

Servo head_pan_servo;
Servo head_tilt_servo;

void setup_servos()
{
	center_servos();
	delay(1000);
	detach_servos();
}

void center_servos()
{
	head_pan_servo.attach(headPanServoPin);
	head_tilt_servo.attach(headTiltServoPin);
	head_pan_servo.write(SONAR_PAN_CENTER);
	head_tilt_servo.write(SONAR_TILT_CENTER);
}

void detach_servos()
{
	head_pan_servo.detach();
	head_tilt_servo.detach();
}

