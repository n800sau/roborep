#include <Arduino.h>
#include "cycle_check.h"

bool cycleCheck(unsigned long *lastMillis, unsigned int cycle)
{
	unsigned long currentMillis = millis();
	bool rs = currentMillis - *lastMillis >= cycle;
	if(rs) *lastMillis = currentMillis;
	return rs;
}
