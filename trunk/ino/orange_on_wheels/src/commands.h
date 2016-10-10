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
	C_RESET_COUNTERS,
	C_WALK_AROUND,
	C_MOVE2RELEASE
};

// reply
enum REPLY {
	R_OK_0 = 1,
	R_ERROR_0,
	R_VOLTS_1F,
	R_MCOUNTS_2F,
	R_MCURRENT_2F, //5
	R_MPOWER_2F,
	R_MDIST_2F,
	R_DIST_1F,     //8
	R_IRDIST_1F,   //9
	R_MOTION_1F,   //10
	R_HEADING_1F,  //11
	R_COMPASS_3F,  //12
	R_ACC_3F,      //13
	R_ACCAVG_3F,   //14
	R_ACCMAX_3F,   //15
	R_HIT_1F,      //16
	R_GYRO_3F,     //17
	R_TEMPERATURE_1F, //18
	R_PRESSURE_1F,    //19
	R_END                // end of block marker
};



#endif //__COMMANDS_H
