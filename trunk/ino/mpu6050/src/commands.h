#ifndef __COMMANDS_H

#define __COMMANDS_H

// commands
enum COMMANDS {
	C_STATE = 0x01
};

// reply
enum REPLY {
	R_OK_0 = 0x01,
	R_ACC_3F,
	R_END                // end of block marker
};



#endif //__COMMANDS_H
