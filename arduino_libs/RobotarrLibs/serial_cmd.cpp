#include <Arduino.h>
#include <Wire.h>
#include "serial_cmd.h"

void serialSetup(int i2c_addr)
{
	Serial.begin(9600);
	//i2c
	Wire.begin(i2c_addr);
}

int processCmdline(const String &cmdline, String c_args[], int n)
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
//			Serial.println(val);
			c_args[i++] = val;
		}
	}
	for(int j = i;j < n; j++) {
		c_args[j] = "";
	}
	return i;
}

int getIntVal(const String &strval)
{
	int rs=0;
	int signm = 1;
	int i=0;
	while(i<strval.length() && strval.charAt(i) != ';' && strval.charAt(i) != 0) {
		char b = strval.charAt(i++);
		if(b == ';') {
			break;
		}
		if(i == 0 && b == '-') {
			signm = -1;
		} else {
			rs = b - '0' + rs * 10;
		}
	}
	rs *= signm;
	return rs;
}

int readSerial(String c_args[], int max_c_args)
{
	static byte cmdbuf[100]="";
	static byte n=0;
	byte nc=0;
	bool finished = false;
	bool error = false;
	while(Serial.available()) {
		byte b = Serial.read();
		if(b == '\xa') {
			b = 0;
			finished = true;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = b;
			if(b == 0) {
				break;
			}
		} else {
			cmdbuf[0] = 0;
			error = true;
			cmdbuf[0] = 0;
			n = 0;
			finished = false;
			break;
		}
	}
	if(finished) {
		nc = processCmdline(String((char*)cmdbuf), c_args, max_c_args);
		cmdbuf[0] = 0;
		n = 0;
	}
	return (error)? -1: nc;
}

int readI2C(String c_args[], int max_c_args)
{
	static byte cmdbuf[100] = "";
	static byte n = 0;
	byte nc = 0;
	bool finished = false;
	bool error = false;
	while(Wire.available()) {
		byte b = Wire.read();
		if(b == '\xa') {
			b = 0;
			finished = true;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = b;
			if(b == 0) {
				break;
			}
		} else {
			cmdbuf[0] = 0;
			error = true;
			cmdbuf[0] = 0;
			n = 0;
			finished = false;
			break;
		}
//		Serial.println(serialData);
//		Serial.flush();
	}
	if(finished) {
		nc = processCmdline(String((char*)cmdbuf), c_args, max_c_args);
		cmdbuf[0] = 0;
		n = 0;
	}
	return (error)? -1: nc;
}

