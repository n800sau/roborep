#ifndef __COMMANDS_H

#define __COMMANDS_H

// commands
enum COMMANDS {
	C_STATE = 0x01,
	C_STOP,
	C_FORWARD,
	C_BACK,
	C_TLEFT,
	C_TRIGHT,
	C_RESCNT,
	C_MCALIB,
	C_SETACC,
	C_SPIN,
	C_TURN2HEAD
};

// reply
enum REPLY {
	R_ACC_X = 0x01
};



#endif //__COMMANDS_H
