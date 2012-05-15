#include "serial_cmd.h"

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
