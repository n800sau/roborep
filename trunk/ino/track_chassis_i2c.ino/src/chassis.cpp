#include <Arduino.h>
#include "pins.h"
#include "chassis.h"

void chassisSetup()
{
	pinMode(LEFT_PWM_PIN, OUTPUT);
	pinMode(LEFT_DIR_PIN, OUTPUT);
	pinMode(RIGHT_PWM_PIN, OUTPUT);
	pinMode(RIGHT_DIR_PIN, OUTPUT);
}

//right
void motorRight(int pwm, boolean reverse)
{
	if (pwm < 0) {
		pwm = -pwm;
		reverse = !reverse;
	}
	analogWrite(RIGHT_PWM_PIN, pwm);
	//set pwm control, 0 for stop, and 255 for maximum speed
	if (reverse) {
		digitalWrite(RIGHT_DIR_PIN, HIGH);
	} else {
		digitalWrite(RIGHT_DIR_PIN, LOW);
	}
}

//left
void motorLeft(int pwm, boolean reverse)
{
	if (pwm < 0) {
		pwm = -pwm;
		reverse = !reverse;
	}
	analogWrite(LEFT_PWM_PIN, pwm);
	if (reverse) {
		digitalWrite(LEFT_DIR_PIN, HIGH);
	} else {
		digitalWrite(LEFT_DIR_PIN, LOW);
	}
}
