#include <WProgram.h>
#include <Wire.h>
#include "serial_cmd.h"

void serialSetup(i2c_addr)
{
	Serial.begin(9600);
	//i2c
	Wire.begin(i2c_addr);
	//join i2c bus 
	Wire.onReceive(I2C_receiveEvent);
	Wire.onRequest(I2C_requestData);
}

int processCmdline(String &cmdline, String c_args[], int n)
{
	String val;
	int lastpos=0, pos=0, i=0;
	while(pos >= 0 && i < n) {
		pos = cmdline.indexOf(';', lastpos);
		if(pos < 0) {
			//the last value
			val = String(cmdline.substring(lastpos));
		} else {
			val = String(cmdline.substring(lastpos, pos));
			lastpos = pos + 1;
		}
		if(val.length() > 0) {
			c_args[i++] = val;
		}
	}
	for(int j = i;j < n; j++) {
		c_args[j] = "";
	}
	return i
}

int getIntVal(const String &strval)
{
	int rs=0;
	int signm = 1;
	int i=0;
	while(i<strval.length() && strval.charAt(i) != ';' && strval.charAt(i) != 0) {
		char b = strval.charAt(i++);
		if(byte == ';') {
			break;
		}
		if(i == 0 && b == '-') {
			signm = -1;
		} else {
			rs = b - '0' + rs * 10;
		}
	}
	rs *= signm;
	Serial.print("value:");
	Serial.println(rs);
	return rs;
}

void readSerial(String &c_args[], int max_c_args)
{
	byte cmdbuf[100];
	byte n = 0, nc=0;
	bool error = false;
	while(Serial.available()) {
		byte b = Serial.read();
		if(b == '\n') {
			b = 0;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = b;
			if(b == 0) {
				break;
			}
		} else {
			cmdbuf[0] = 0;
			error = true;
			break;
		}
//		Serial.println(serialData);
//		Serial.flush();
	}
	if(n > 0 && cmdbuf[0]) {
		nc = processCmdline(String(cmdbuf), c_args, max_c_args);
	}
	return (error)? -1: nc;
}

void readI2C(String &c_args[], int max_c_args)
{
	byte cmdbuf[100];
	byte n = 0, nc=0;
	bool error = false;
	while(Wire.available()) {
		byte b = Wire.receive();
		if(b == '\n') {
			b = 0;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = b;
			if(b == 0) {
				break;
			}
		} else {
			cmdbuf[0] = 0;
			error = true;
			break;
		}
//		Serial.println(serialData);
//		Serial.flush();
	}
	if(n > 0 && cmdbuf[0]) {
		nc = processCmdline(String(cmdbuf), c_args, max_c_args);
	}
	return (error)? -1: nc;
}

struct {
	char cmd;
} I2C_request = {NULL};

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void I2C_receiveEvent(int howMany)
{
	String c_args[MAX_CMDARGS];
	int nc = readI2C(c_args, MAX_CMDARGS);
	if(nc > 0) {
		executeCommand(c_args, nc);
	}
}

void I2C_requestData()
{
	if(I2C_request.cmd) {
		Wire.beginTransmission(I2C_TopModule_Addr); // transmit to Top Module
		Wire.send("ok");
		Wire.endTransmission();       // stop transmitting
		I2C_request.cmd = NULL;
	}
}
