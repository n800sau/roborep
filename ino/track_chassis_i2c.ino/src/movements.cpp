#include <Arduino.h>
#include "pins.h"
#include "movements.h"
#include "servo_x.h"

const MOVEMENT *current_move = NULL;

ServoX baseturn_servo, basetilt_servo, middletilt_servo;

void movementsSetup()
{
	//servos
	basetilt_servo.attach(BASE_TILT_PWM_PIN);
	baseturn_servo.attach(BASE_TURN_PWM_PIN);
	middletilt_servo.attach(MIDDLE_TILT_PWM_PIN);
}

bool set_movement(String mname)
{
	bool found = false;
	for(int i = 0; i < sizeof(sizeof(moves)/sizeof(moves[0])); i++)
	{
		const MOVEMENT *m = &moves[i];
		if(m->name == mname) {
			baseturn_servo.setAngle(m->angle[BASETURN], m->speeds[BASETURN]);
			basetilt_servo.setAngle(m->angle[BASETILT], m->speeds[BASETILT]);
			middletilt_servo.setAngle(m->angle[MIDDLETILT], m->speeds[MIDDLETILT]);
			found = true;
			current_move = m;
			break;
		}
	}
	return found;
}

void movement_stop()
{
	basetilt_servo.stop();
	baseturn_servo.stop();
	middletilt_servo.stop();
}

bool movementsUpdate()
{
	return baseturn_servo.update() && basetilt_servo.update() && middletilt_servo.update();
}

void get_angles(byte angles[3])
{
	angles[BASETURN] = baseturn_servo.read();
	angles[BASETILT] = basetilt_servo.read();
	angles[MIDDLETILT] = middletilt_servo.read();
}

