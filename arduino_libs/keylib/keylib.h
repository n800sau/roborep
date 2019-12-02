#ifndef KEYLIB_H

#define KEYLIB_H

#include <Arduino.h>

class readKey
{
	protected:

		int icount;
		int ocount;
		const uint8_t *poline;
		const uint8_t *piline;

		byte last_key;
		unsigned long  last_change_millis;
		unsigned long debounce_millis;

	public:
		readKey();
		void begin(int ocount, int icount, const uint8_t *poline,  const uint8_t *piline, unsigned long debounce_millis=500);
		byte read_key();
		byte read_key_debounce();

};

#endif // KEYLIB_H
