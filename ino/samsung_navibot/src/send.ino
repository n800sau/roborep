#include <IRLibSendBase.h>	// First include the send base
//Now include only the protocols you wish to actually use.
//The lowest numbered protocol should be first but remainder
//can be any order.
#include <IRLib_P07_NECx.h>
//#include <IRLib_P08_Samsung36.h>
#include <IRLibCombo.h>	 // After all protocols, include this
// All of the above automatically creates a universal sending
// class called "IRsend" containing only the protocols you want.
// Now declare an instance of that sender.

#define CODE_POWER 0x818100FFUL
#define CODE_RECHARGING 0x8181807FUL
#define CODE_AUTO 0x818140BFUL
#define CODE_SPOT 0x8181C03FUL
#define CODE_MAX 0x818120DFUL
#define CODE_START_STOP 0x818110EFUL
#define CODE_LEFT 0x8181609FUL
#define CODE_RIGHT 0x8181E01FUL
#define CODE_FORWARD 0x8181A05FUL
#define CODE_MANUAL 0x8181906FUL

#define CMD_COUNT 10

const struct {
	int b;
	unsigned long code;
	String descr;
} cmds[CMD_COUNT] = {
	{' ', CODE_POWER, "power"},
	{'c', CODE_RECHARGING, "recharging"},
	{'a', CODE_AUTO, "auto"},
	{'s', CODE_SPOT, "spot"},
	{'x', CODE_MAX, "max"},
	{'g', CODE_START_STOP, "start/stop"},
	{'l', CODE_LEFT, "left"},
	{'r', CODE_RIGHT, "right"},
	{'f', CODE_FORWARD, "forward"},
	{'m', CODE_MANUAL, "manual"}
};

IRsend mySender;

void setup() {
	Serial.begin(115200);
	delay(2000); while (!Serial); //delay for Leonardo
	Serial.println(F("Commands:"));
	for(int i=0; i<CMD_COUNT; i++) {
		Serial.print(F("'"));
		Serial.print((char)cmds[i].b);
		Serial.print(F("' - "));
		Serial.print(cmds[i].descr);
		Serial.print(F(" (0x"));
		Serial.print(cmds[i].code, HEX);
		Serial.println(F(")"));
	}
}

void loop()
{
	int i, p;
	int c = Serial.read();
	if ( c != -1) {
		p = -1;
		for(i=0; i<CMD_COUNT; i++) {
			if(c == cmds[i].b) {
				p = i;
				break;
			}
		}
		if(p >= 0) {
			//send a code every time a right character is received from the
			// serial port
			for(int i=0;i<5;i++) {
				mySender.send(NECX, cmds[p].code);
				delay(10);
//				mySender.send(SAMSUNG36, cmds[p].code);
			}
			Serial.print(F("Sent signal \""));
			Serial.print(cmds[p].descr);
			Serial.print(F("\" (0x"));
			Serial.print(cmds[p].code, HEX);
			Serial.println(F(")"));
		}
	}
}

