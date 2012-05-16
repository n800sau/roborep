#include <WProgram.h>
#include "pins.h"
#include "movements.h"
#include "servo_x.h"

MOVEMENT *current_move = NULL;

ServoX palmturn_servo, palmtilt_servo, claw_servo;

void movementsSetup()
{
	//servos
	palmtilt_servo.attach(PALM_TILT_PWM_PIN);
	palmturn_servo.attach(PALM_TURN_PWM_PIN);
	claw_servo.attach(CLAW_PWM_PIN);
}

bool set_movement(const char *mname)
{
	bool found = false;
	for(int i = 0; i < sizeof(moves/sizeof(moves[0])); i++)
	{
		MOVEMENT *m = moves[i];
		if(strcmp(m->name, mname) == 0) {
			palmturn_servo.setAngle(m->angle[PALMTURN], m->speed[PALMTURN]);
			palmtilt_servo.setAngle(m->angle[PALMTILT], m->speed[PALMTILT]);
			claw_servo.setAngle(m->angle[CLAW], m->speed[CLAW]);
			found = true;
			current_move = m;
			break;
		}
	}
	return found;
}

bool movementsUpdate()
{
	return palmturn_servo.update() && palmtilt_servo.update() && claw_servo.update();
}
