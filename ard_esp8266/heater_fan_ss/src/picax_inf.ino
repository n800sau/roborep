#include <SoftwareSerial.h>

// on commands <param: val>
const int P1 = 0x81;
const int P2 = 0x82;
const int P3 = 0x83;

// off commands
const int S1 = 0x91;
const int S2 = 0x92;
const int S3 = 0x93;

// high commands
const int PH1 = 0xA1;
const int PH2 = 0xA2;
const int PH3 = 0xA3;

// low commands
const int PL1 = 0xB1;
const int PL2 = 0xB2;
const int PL3 = 0xB3;

// get data
const int GD = 0xC1;

const int out_pin = 13;
const int in_pin = 12;

const int cmd_seq[] = {
	P1, P2, P3,
	S1, S2, S3,
	PH1, PH2, PH3,
	PL1, PL2, PL3,
	GD
};

SoftwareSerial sSerial(in_pin, out_pin); // RX, TX


void setup()
{
	Serial.begin(115200);
	sSerial.begin(2400);

	// calibrate

}

int i = 0;
int j = 0;
void loop()
{
	Serial.print("Sending ");
	Serial.println(i);
	int cmd = cmd_seq[i++];
	if(i >= sizeof(cmd_seq)/sizeof(cmd_seq[0])) {
		i = 0;
	}
	sSerial.print("CMD");
	sSerial.write(cmd);
	delay(100);
	sSerial.write(j++%255);
	delay(1000);
	while(sSerial.available()) {
		Serial.print(sSerial.read());
		Serial.print(" ");
		delay(100);
	}
	Serial.println();
	delay(1000);
}
