#ifndef __MOVEMENTS_CMD__

#define __MOVEMENTS_CMD__

enum TSERVO {
	BASETURN=0, BASETILT, MIDDLETILT, TSERVONUM
};

typedef struct {
	const char *name;
	byte angle[TSERVONUM];
	byte speeds[TSERVONUM];
} MOVEMENT;

const MOVEMENT moves[] = {
	{"base", {90, 0, 180}, {10, 10, 10}},
	{"candle", {90, 90, 90}, {10, 10, 10}}
};

extern MOVEMENT *current_move;

extern ServoX baseturn_servo, basetilt_servo, middletilt_servo;

void movementsSetup();
bool movementsUpdate();

void set_movement(const char *mname);

#endif //__MOVEMENTS_CMD__
