#include "motion.h"

const unsigned long IR_UP = 0x10;
const unsigned long IR_DOWN = 0x16;
const unsigned long IR_LEFT = 0x12;
const unsigned long IR_RIGHT = 0x14;
const unsigned long IR_STOP = 0x13;
const unsigned long IR_L = 0x0C;
const unsigned long IR_ENTER = 0x0D;
const unsigned long IR_R = 0x0E;
const unsigned long IR_0 = 0x03;
const unsigned long IR_2 = 0x04;
const unsigned long IR_3 = 0x05;
const unsigned long IR_4 = 0x06;
const unsigned long IR_5 = 0x07;
const unsigned long IR_6 = 0x08;
const unsigned long IR_7 = 0x09;
const unsigned long IR_8 = 0x0A;
const unsigned long IR_9 = 0x0B;
const unsigned long IR_RED = 0x1C;
const unsigned long IR_YELLOW = 0x1D;
const unsigned long IR_GREEN = 0x1F;
const unsigned long IR_BLUE = 0x20;

void evIRcmd()
{
	if (Serial3.available()) {
		int inByte = Serial3.read();
		nh.loginfo(("Byte:" + String(inByte, HEX)).c_str());
		switch(inByte) {
			case IR_UP:
				motion::set_motion(0.1, 0);
				break;
			case IR_DOWN:
				motion::set_motion(-0.1, 0);
				break;
			case IR_RIGHT:
				motion::set_motion(0, 0.1);
				break;
			case IR_LEFT:
				motion::set_motion(0, -0.1);
				break;
			case IR_STOP:
				motion::stop();
				break;
		}
	}
}
