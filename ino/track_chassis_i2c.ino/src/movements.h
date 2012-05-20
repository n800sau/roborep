#ifndef __MOVEMENTS_CMD__

#define __MOVEMENTS_CMD__

#include "servo_x.h"

enum TSERVO {
	BASETURN=0, BASETILT, MIDDLETILT, TSERVONUM
};

typedef struct {
	String name;
	byte angle[TSERVONUM];
	byte speeds[TSERVONUM];
} MOVEMENT;

const MOVEMENT moves[] = {
	{"base", {90, 0, 90}, {10, 10, 10}},
	{"candle", {90, 90, 90}, {10, 10, 10}}
};

extern const MOVEMENT *current_move;

extern ServoX baseturn_servo, basetilt_servo, middletilt_servo;

void movementsSetup();
bool movementsUpdate();

bool set_movement(String mname);
void movement_stop();
void get_angles(byte angles[3]);

#endif //__MOVEMENTS_CMD__
