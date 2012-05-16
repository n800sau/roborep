#ifndef __MOVEMENTS_CMD__

#define __MOVEMENTS_CMD__

enum TSERVO {
	PALMTURN=0, PALMTILT, CLAW, TSERVONUM
};

typedef struct {
	const char *name;
	byte angle[TSERVONUM];
	byte speeds[TSERVONUM];
} MOVEMENT;

const MOVEMENT moves[] = {
	{"openclaw", {-1, -1, 0}, {10, 10, 10}},
	{"closeclaw", {-1, -1, 180}, {10, 10, 10}}
};

extern MOVEMENT *current_move;

extern ServoX palmturn_servo, palmtilt_servo, claw_servo;

void movementsSetup();
bool movementsUpdate();

void set_movement(const char *mname);

#endif //__MOVEMENTS_CMD__
