#ifndef __MEGA_PROGRAMMER_H
#define __MEGA_PROGRAMMER_H

#include <Arduino.h>

#define PAGE_SIZE (8*16)
#define VALS_COUNT PAGE_SIZE

struct _CUR_LINE {
	unsigned addr;
	unsigned type;
	int n;
	unsigned vals[VALS_COUNT];
	int idx;
};

class MegaProgrammer {

		HardwareSerial *pSerial;
		int pin_reset;
		_CUR_LINE cline;
		bool end_of_data;

	protected:

		int waitForBytes(int count);
		bool in_sync(char fb, char lb);
		void read_next_line();
		int read_byte();

	public:

		MegaProgrammer() {}

		void begin(HardwareSerial &serial, int reset_pin, long slave_baud=115200);
		void stk500program();
		void resetSlave();
		void pass_through_exchange();
		void flushOutSerial();
		void setBaud(unsigned long baud);

};

#endif // __MEGA_PROGRAMMER_H
