#ifndef __COMMANDS_H

#define __COMMANDS_H

// commands
enum COMMANDS {
	C_STATE = 0x01,
	C_MLEFT,
	C_MRIGHT,
	C_MBOTH,
	C_MSTOP,
	C_RESET_COUNTERS
};

// reply
enum REPLY {
	R_OK_0 = 0x01,
	R_ERROR_0,
	R_VOLTS_1F,
	R_MCOUNTS_2F,
	R_MDIST_2F,
	R_DIST_1F,
	R_END                // end of block marker
};



#endif //__COMMANDS_H
