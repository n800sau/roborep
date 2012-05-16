#ifndef __COMMANDS_CMD__

#define __COMMANDS_CMD__

enum SCOMMANDS {
	CMD_LEFT,
	CMD_RIGHT,
	CMD_BOTH,
	CMD_STOP,
	CMD_BATTERY,
	CMD_CHARGER,
	CMD_CURRENT,
	CMD_LED,
	CMD_MOVEMENT,
	CMD_SETBASETURN,
	CMD_SETBASETILT,
	CMD_SETMIDDLETILT,
};

//long string commands
typedef struct {
	const char *cmdstr;
	SCOMMANDS cmd;
	int n_parms;
} LONG_CMD;

const LONG_CMD scommands[] = {
	{"left", CMD_LEFT, 1},
	{"right", CMD_RIGHT, 1},
	{"both", CMD_BOTH, 2},
	{"stop", CMD_STOP, 0},
	{"battery", CMD_BATTERY, 0},
	{"charger", CMD_CHARGER, 0},
	{"current", CMD_CURRENT, 0},
	{"led", CMD_LED, 1},
	{"move", CMD_MOVEMENT, 1},
	{"ge", CMD_SHOWGE, 0},
	{"setBaseTurn", CMD_SETBASETURN, 2},
	{"setBaseTilt", CMD_SETBASETILT, 2},
	{"setMiddleTilt", CMD_SETMIDDLETILT, 2}
};

#endif //__COMMANDS_CMD__
