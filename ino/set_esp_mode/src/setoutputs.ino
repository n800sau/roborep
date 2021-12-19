#include "Wire.h"

#include <INA219.h>
INA219 monitor;

const int boot_pin = 11;
const int reset_pin = 12;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);

	monitor.begin(64);
	monitor.configure(0, 3, 3, 7);

	// test shunt = 115mm of 22AWG solid copper = 0.3 Ohms
	monitor.calibrate(0.3, 0.2, 6, 0.25);

	// initialize the digital pin as input (work mode)
	pinMode(boot_pin, INPUT_PULLUP);
	pinMode(reset_pin, INPUT_PULLUP);
	Serial.println("Ready");
}

void info()
{
//	Serial.print("raw shunt voltage: ");
//	Serial.println(monitor.shuntVoltageRaw());

//	Serial.print("raw bus voltage: ");
//	Serial.println(monitor.busVoltageRaw());

//	Serial.println("--");

	Serial.print("shunt voltage: ");
	Serial.print(monitor.shuntVoltage() * 1000, 4);
	Serial.println(" mV");

	Serial.print("shunt current: ");
	Serial.print(monitor.shuntCurrent() * 1000, 4);
	Serial.println(" mA");

	Serial.print("bus voltage: ");
	Serial.print(monitor.busVoltage(), 4);
	Serial.println(" V");

	Serial.print("bus power: ");
	Serial.print(monitor.busPower() * 1000, 4);
	Serial.println(" mW");
}

void loop() {
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == 'W' || incomingByte == 'w') {
			// work mode
			pinMode(boot_pin, INPUT_PULLUP);
//			digitalWrite(boot_pin, HIGH);
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT_PULLUP);
//			digitalWrite(reset_pin, HIGH);
			Serial.println("W mode");
			info();
			Serial.println("end");
		} else if (incomingByte == 'P' || incomingByte == 'p') {
			// programming mode
			pinMode(boot_pin, OUTPUT);
			digitalWrite(boot_pin, LOW);
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT);
//			digitalWrite(reset_pin, HIGH);
			Serial.println("P mode");
			info();
			Serial.println("end");
		} else if (incomingByte == 'R' || incomingByte == 'r') {
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT_PULLUP);
			digitalWrite(reset_pin, HIGH);
			Serial.println("Reset");
			Serial.println("end");
		} else if (incomingByte == 'I' || incomingByte == 'i') {
			Serial.println("Info:");
			info();
			Serial.println("end");
		} else if (incomingByte == '?') {
			Serial.println("Ready");
		}
	}
}
