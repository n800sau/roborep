#ifndef __MOVEMENTS_CMD__

#define __MOVEMENTS_CMD__

#include <Arduino.h>
#include "servo_x.h"

enum TSERVO {
	PALMTURN=0, PALMTILT, CLAW, TSERVONUM
};

typedef struct {
	String name;
	byte angle[TSERVONUM];
	byte speeds[TSERVONUM];
} MOVEMENT;

const MOVEMENT moves[] = {
	{"openclaw", {90, 90, 90}, {10, 10, 120}},
	{"closeclaw", {90, 90, 180}, {10, 10, 120}}
};

extern const MOVEMENT *current_move;

extern ServoX palmturn_servo, palmtilt_servo, claw_servo;

void movementsSetup();
bool movementsUpdate();

bool set_movement(String mname);
void movement_stop();
void get_angles(byte angles[3]);

#endif //__MOVEMENTS_CMD__
