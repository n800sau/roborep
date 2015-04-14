#include "atexec.h"

typedef struct config_commands {
	String command;
	void (*function)(AtExec *self);
} config_commands_t;

const config_commands_t config_commands[] = { 
	{ "HELP", &AtExec::print_help },
	{ "BAUD", &AtExec::config_baud },
//	{ "IFCONFIG", &do_ifconfig },
//	{ "RESET", &config_cmd_reset }, 
//	{ "MODE", &config_cmd_mode },
//	{ "STA", &config_cmd_sta },
//	{ "AP", &config_cmd_ap },
//	{ "IORST", &io_reset },
//	{ "DEBUG", &config_debug_mode },
	{ NULL, NULL }
};

void AtExec::parseCommand(String &line) {

	Serial.print("parse command=");
	Serial.println(line);

	if(line.startsWith("+++AT")) {

		int pos = 5;

		while(pos > -1) {
			int nextpos = line.indexOf(" ", pos+1);
			String el = ((nextpos < 0) ? line.substring(pos) : line.substring(pos, nextpos));
			el.trim();
			if(el.length() > 0) {
				argv.push(el);
			}
			pos = nextpos;
		}

		if(argv.count() > 0) {
			String cmd = argv.get(0);
			for (int i = 0; config_commands[i].command; ++i) {
				if (cmd.equals(config_commands[i].command)) {
					config_commands[i].function(this);
					break;
				}
			}
		}
	}

/*	if(cmd.equals("quit")) {
		client.stop();
	} else if(cmd.equals("help")) {
	} else if(cmd.equals("on")) {
		digitalWrite(5, HIGH);
		client.println("set on");
	} else if(cmd.equals("off")) {
		digitalWrite(5, LOW);
		client.println("set off");
	} else {
		client.print("Invalid command:");
		client.println(cmd);
	}*/
}

void AtExec::print_help(AtExec *self) {
	self->client.println("--- Telnet Server Help ---");
	self->client.println("on    : switch on the Main Power");
	self->client.println("off   : switch off the Main Power");
	self->client.println("quit  : close the connection");
}

void AtExec::config_baud(AtExec *self) {
	for(int i=0; i<self->argv.count(); i++) {
		Serial.println(self->argv.get(i));
	}
//	Serial.end();
//	Serial.begin(115200);
}

