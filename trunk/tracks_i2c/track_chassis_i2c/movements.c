#include <WProgram.h>
#include "pins.h"
#include "movements.h"
#include "servo_x.h"

MOVEMENT *current_move = NULL;

ServoX baseturn_servo, basetilt_servo, middletilt_servo;

void movementsSetup()
{
	//servos
	basetilt_servo.attach(BASE_TILT_PWM_PIN);
	baseturn_servo.attach(BASE_TURN_PWM_PIN);
	middletilt_servo.attach(MIDDLE_TILT_PWM_PIN);
}

bool set_movement(const char *mname)
{
	bool found = false;
	for(int i = 0; i < sizeof(moves/sizeof(moves[0])); i++)
	{
		MOVEMENT *m = moves[i];
		if(strcmp(m->name, mname) == 0) {
			baseturn_servo.setAngle(m->angle[BASETURN], m->speed[BASETURN]);
			basetilt_servo.setAngle(m->angle[BASETILT], m->speed[BASETILT]);
			middletilt_servo.setAngle(m->angle[MIDDLETILT], m->speed[MIDDLETILT]);
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

void get_angles(byte &angles[3])
{
	angles[BASETURN] = palmturn_servo.read();
	angles[BASETILT] = palmtilt_servo.read();
	angles[MIDDLETILT] = claw_servo.read();
}

