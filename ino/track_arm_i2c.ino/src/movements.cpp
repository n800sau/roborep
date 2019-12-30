#include <WString.h>
#include "pins.h"
#include "movements.h"
#include "servo_x.h"

const MOVEMENT *current_move = NULL;

ServoX palmturn_servo, palmtilt_servo, claw_servo;

void movementsSetup()
{
	//servos
	palmtilt_servo.attach(PALM_TILT_PWM_PIN);
	palmturn_servo.attach(PALM_TURN_PWM_PIN);
	claw_servo.attach(CLAW_PWM_PIN);
}

bool set_movement(String mname)
{
	bool found = false;
	for(int i = 0; i < sizeof(sizeof(moves)/sizeof(moves[0])); i++)
	{
		const MOVEMENT *m = &moves[i];
		if(m->name == mname) {
			palmturn_servo.setAngle(m->angle[PALMTURN], m->speeds[PALMTURN]);
			palmtilt_servo.setAngle(m->angle[PALMTILT], m->speeds[PALMTILT]);
			claw_servo.setAngle(m->angle[CLAW], m->speeds[CLAW]);
			found = true;
			current_move = m;
			break;
		}
	}
	return found;
}

void movement_stop()
{
	palmtilt_servo.stop();
	palmturn_servo.stop();
	claw_servo.stop();
	current_move = NULL;
}

bool movementsUpdate()
{
	return palmturn_servo.update() && palmtilt_servo.update() && claw_servo.update();
}

void get_angles(byte angles[3])
{
	angles[PALMTURN] = palmturn_servo.read();
	angles[PALMTILT] = palmtilt_servo.read();
	angles[CLAW] = claw_servo.read();
}

