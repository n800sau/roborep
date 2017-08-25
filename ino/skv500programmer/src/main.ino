#include <mega_programmer.h>

#define RESET_PIN_COUNT 3
const int pin_reset_list[RESET_PIN_COUNT] = {40, 38, 36};
MegaProgrammer programmers[3];

int curprog_idx = 0;
unsigned long slave_baud = 115200;

void setup()
{
	Serial.begin(115200);
	Serial.println("Ready");
	programmers[0].begin(Serial1, pin_reset_list[0]);
	programmers[1].begin(Serial2, pin_reset_list[1]);
	programmers[2].begin(Serial3, pin_reset_list[2]);
	resetOutSerial();
}


bool pass_through_mode = false;

void resetOutSerial()
{
	programmers[curprog_idx].setBaud(slave_baud);
	Serial.print("Slave is Serial ");
	Serial.print(curprog_idx + 1);
	Serial.print(", baud: ");
	Serial.println(slave_baud);
}

void loop()
{
	if(pass_through_mode) {
		programmers[curprog_idx].pass_through_exchange();
	} else {
		if(Serial.available()) {
			int cmd = Serial.read();
			switch(cmd)
			{
				case '!':
					curprog_idx = 0;
					resetOutSerial();
					break;
				case '@':
					curprog_idx = 1;
					resetOutSerial();
					break;
				case '#':
					curprog_idx = 2;
					resetOutSerial();
					break;
				case '1':
					slave_baud = 19200;
					resetOutSerial();
					break;
				case '2':
					slave_baud = 38400;
					resetOutSerial();
					break;
				case '3':
					slave_baud = 57600;
					resetOutSerial();
					break;
				case '4':
					slave_baud = 115200;
					resetOutSerial();
					break;
				case 'c':
					resetOutSerial();
					programmers[curprog_idx].stk500program();
					break;
				case 'P':
				case 'p':
					resetOutSerial();
//					if(cmd == 'P') {
						programmers[curprog_idx].resetSlave();
//					}
					programmers[curprog_idx].flushOutSerial();
					pass_through_mode = true;
					Serial.println("Pass through mode is set");
					break;
			}
		}
	}
}
