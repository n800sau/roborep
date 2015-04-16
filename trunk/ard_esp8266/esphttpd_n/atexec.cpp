#include "atexec.h"
#include "dbgserial.h"
#include <Esp.h>
#include <ESP8266WiFi.h>

#define ARDUINO_RESET_PIN 12

typedef struct config_commands {
	String command;
	void (*function)(AtExec *self);
} config_commands_t;

const config_commands_t config_commands[] = { 
	{ "HELP", &AtExec::print_help },
	{ "BAUD", &AtExec::config_baud },
	{ "NETINFO", &AtExec::show_netinfo },
	{ "RESET", &AtExec::cmd_reset }, 
	{ "MODE", &AtExec::config_cmd_mode },
//	{ "STA", &config_cmd_sta },
//	{ "AP", &config_cmd_ap },
	{ "IORST", &AtExec::io_reset },
	{ "", &AtExec::unknown_command },
//	{ "DEBUG", &config_debug_mode },
	{ NULL, NULL }
};

AtExec::AtExec(WiFiClient client):
	client(client),argv(false)
{
	pinMode(ARDUINO_RESET_PIN, OUTPUT);
	digitalWrite(ARDUINO_RESET_PIN, HIGH);
}

void AtExec::ok()
{
	client.println("OK");
}

void AtExec::error()
{
	client.println("ERROR");
}

bool AtExec::parseCommand(String &line)
{
	bool rs = false;

	dbgSerial.print("parse command=");
	dbgSerial.println(line);

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
			int i;
			for (i = 0; config_commands[i].command; ++i) {
				if (cmd.equals(config_commands[i].command)) {
					config_commands[i].function(this);
					rs = true;
					break;
				}
			}
			if(!config_commands[i].command) {
				client.print("Invalid command:");
				client.println(line);
				error();
			}
		} else {
			ok();
		}
	}
	return rs;
}

void AtExec::print_help(AtExec *self)
{
	self->client.println("--- epshttpd_n help ---");
	const config_commands_t *p = config_commands;
	char *buf = NULL;
	while(p->command) {
		self->client.println(p->command);
		p++;
	}
	self->ok();
}

void AtExec::config_baud(AtExec *self)
{
	if(self->argv.count() == 1) {
		self->error();
	} else {
		Serial.end();
		Serial.begin(self->argv.get(1).toInt());
		self->ok();
	}
}

void AtExec::config_cmd_mode(AtExec *self)
{
	if(self->argv.count() == 1) {
		self->error();
	} else {
		WiFiMode mode;
		String s_mode = self->argv.get(1);
		if(s_mode == "STA") {
			mode = WIFI_STA;
		} else if(s_mode == "AP") {
			mode = WIFI_AP;
		} else if(s_mode == "APSTA") {
			mode = WIFI_AP_STA;
		} else {
			s_mode = "";
		}
		if(s_mode != "") {
			WiFi.mode(mode);
			self->ok();
		} else {
			self->client.println("Mode should be AP, STA or APSTA");
			self->error();
		}
	}
}

void AtExec::cmd_reset(AtExec *self)
{
	self->ok();
	delay(500);
	ESP.reset();
}

void AtExec::io_reset(AtExec *self)
{
	digitalWrite(ARDUINO_RESET_PIN, LOW);
	delay(500);
	digitalWrite(ARDUINO_RESET_PIN, HIGH);
	self->ok();
}

void AtExec::show_netinfo(AtExec *self)
{
	WiFi.printDiag(self->client);
	self->ok();
}

void AtExec::unknown_command(AtExec *self)
{
	self->error();
}

