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
	C_TURN2HEAD  // uint16_t n degrese parametre
};

// reply
enum REPLY {
	R_OK_0 = 0x01,
	R_ACC_3F,
	R_VOLTS_1F,
	R_MCOUNTS_2F,
	R_HEADING_1F,
	R_ACCAVG_3F,
	R_ACCMAX_3F,
	R_HIT_1F,
	R_GYRO_3F,
	R_TEMPERATURE_1F,
	R_PRESSURE_1F,
	R_ALT_1F,
	R_DISTANCE_1F,
	R_PR_1F,
	R_MCOEF_2F,
	R_VEC_OVERFLOW_1F,
	R_VECTOR_3F,         //lcount, rcount, heading
	R_END                // end of block marker
};



#endif //__COMMANDS_H
