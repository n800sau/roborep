#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Cmd.h"

// command line structure
typedef struct _cmd_t
{
    char *cmd;
    void (*func)(int argc, char **argv);
    struct _cmd_t *next;
} cmd_t;

// command line message buffer and pointer
static char msg[MAX_MSG_SIZE];
static int msg_pos = 0;

// linked list for command table
static cmd_t *cmd_tbl_list, *cmd_tbl;

const char cmd_unrecog[] PROGMEM = "{\"error\": \"Command not recognized\"}";

static Stream* stream;

/**************************************************************************/
/*!
	Parse the command line. This function tokenizes the command input, then
	searches for the command table entry associated with the commmand. Once found,
	it will jump to the corresponding function.
*/
/**************************************************************************/
void cmd_parse(char *cmd)
{
	uint8_t argc, i = 0;
	char *argv[30];
	char buf[50];
	cmd_t *cmd_entry;

	fflush(stdout);

	// parse the command line statement and break it up into space-delimited
	// strings. the array of strings will be saved in the argv array.
	argv[i] = strtok(cmd, " ");
	do
	{
		argv[++i] = strtok(NULL, " ");
	} while ((i < 30) && (argv[i] != NULL));

	// save off the number of arguments for the particular command.
	argc = i;

	// parse the command table for valid command. used argv[0] which is the
	// actual command name typed in at the prompt
	for (cmd_entry = cmd_tbl; cmd_entry != NULL; cmd_entry = cmd_entry->next)
	{
		if (!strcmp(argv[0], cmd_entry->cmd))
		{
			cmd_entry->func(argc, argv);
			return;
		}
	}

	// command not recognized. print message and re-generate prompt.
	strcpy_P(buf, cmd_unrecog);
	stream->println(buf);

}

/**************************************************************************/
/*!
	This function processes the individual characters typed into the command
	prompt. It saves them off into the message buffer unless its a "backspace"
	or "enter" key.
*/
/**************************************************************************/
void cmd_handler()
{
	char c = stream->read();

	switch (c)
	{
	case '.':
	case '\r':
		// terminate the msg and reset the msg ptr. then send
		// it to the handler for processing.
		msg[msg_pos] = 0;
//		stream->print("\r\n");
		cmd_parse(msg);
		msg_pos = 0;
		break;

	case '\n':
		// ignore newline characters. they usually come in pairs
		// with the \r characters we use for newline detection.
		break;

	default:
		// normal character entered. add it to the buffer
//		stream->print(c);
		if(msg_pos < MAX_MSG_SIZE - 1) {
			msg[msg_pos++] = c;
		}
		break;
	}
}

/**************************************************************************/
/*!
	This function should be set inside the main loop. It needs to be called
	constantly to check if there is any available input at the command prompt.
*/
/**************************************************************************/
void cmdPoll()
{
	while (stream->available())
	{
		cmd_handler();
	}
}

/**************************************************************************/
/*!
	Initialize the command line interface. This sets the terminal speed and
	and initializes things.
*/
/**************************************************************************/
void cmdInit(Stream *str)
{
	stream = str;
	// init the msg ptr
	msg_pos = 0;

	// init the command table
	cmd_tbl_list = NULL;

}

/**************************************************************************/
/*!
	Add a command to the command table. The commands should be added in
	at the setup() portion of the sketch.
*/
/**************************************************************************/
void cmdAdd(const char *name, void (*func)(int argc, char **argv))
{
	// alloc memory for command struct
	cmd_tbl = (cmd_t *)malloc(sizeof(cmd_t));

	// alloc memory for command name
	char *cmd_name = (char *)malloc(strlen(name)+1);

	// copy command name
	strcpy(cmd_name, name);

	// fill out structure
	cmd_tbl->cmd = cmd_name;
	cmd_tbl->func = func;
	cmd_tbl->next = cmd_tbl_list;
	cmd_tbl_list = cmd_tbl;
}

/**************************************************************************/
/*!
	Get a pointer to the stream used by the interpreter. This allows
	commands to use the same communication channel as the interpreter
	without tracking it in the main program.
*/
/**************************************************************************/
Stream* cmdGetStream(void)
{
	return stream;
}
