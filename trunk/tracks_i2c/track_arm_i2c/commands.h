#ifndef __COMMANDS_CMD__

#define __COMMANDS_CMD__

enum SCOMMANDS {
	CMD_MOVEMENT,
	CMD_MOVEMENT_STOP,
	CMD_CHASSIS_MOVEMENT,
	CMD_SETPALMTURN,
	CMD_SETPALMTILT,
	CMD_SETCLAW,
};

//long string commands
typedef struct {
	const char *cmdstr;
	SCOMMANDS cmd;
	int n_parms;
} LONG_CMD;

const LONG_CMD scommands[] = {
	{"led", CMD_LED, 1},
	{"move", CMD_MOVEMENT, 1},
	{"move_stop", CMD_MOVEMENT_STOP, 0},
	{"stop", CMD_STOP, 1},
	{"chassis", CMD_CHASSIS, -1},
	{"setPalmTurn", CMD_SETPALMTURN, 2},
	{"setPalmTilt", CMD_SETPALMTILT, 2},
	{"setClaw", CMD_SETCLAW, 2}
};

#endif //__COMMANDS_CMD__
