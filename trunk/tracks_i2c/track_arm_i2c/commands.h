#ifndef __COMMANDS_CMD__

#define __COMMANDS_CMD__

enum SCOMMANDS {
	CMD_MOVEMENT,
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
	{"setPalmTurn", CMD_SETPALMTURN, 2},
	{"setPalmTilt", CMD_SETPALMTILT, 2},
	{"setClaw", CMD_SETCLAW, 2}
};

#endif //__COMMANDS_CMD__
