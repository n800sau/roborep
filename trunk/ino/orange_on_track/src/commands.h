#ifndef __COMMANDS_H

#define __COMMANDS_H

// commands
enum COMMANDS {
	C_PING = 1,
	C_STATE,
	C_MLEFT,
	C_MRIGHT,
	C_MBOTH,
	C_MSTOP,
	C_RESET_COUNTERS
};

// reply
enum REPLY {
	R_OK_0 = 1,
	R_ERROR_0,
	R_VOLTS_1F,
	R_MCOUNTS_2F,
	R_MCURRENT_2F,
	R_MPOWER_2F,
	R_MDIST_2F,
	R_DIST_1F,
	R_END                // end of block marker
};



#endif //__COMMANDS_H