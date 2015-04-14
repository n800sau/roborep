#include "atexec.h"

void AtExec::parseCommand(String &cmd) {
	Serial.print("parse command=");
	Serial.println(cmd);

	if(cmd.equals("quit")) {
		client.stop();
	} else if(cmd.equals("help")) {
		Serial.println("Help output");
		client.println("--- Telnet Server Help ---");
		client.println("on    : switch on the Main Power");
		client.println("off   : switch off the Main Power");
		client.println("quit  : close the connection");
	} else if(cmd.equals("on")) {
		digitalWrite(5, HIGH);
		client.println("set on");
	} else if(cmd.equals("off")) {
		digitalWrite(5, LOW);
		client.println("set off");
	} else {
		client.print("Invalid command:");
		client.println(cmd);
	}
}
