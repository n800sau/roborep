#include "encoders.h"
#include "const.h"

namespace encoders {

	const int PIN_LEFT = 2;
	const int PIN_RIGHT = 3;


	volatile int lCounter = 0;
	volatile int rCounter = 0;
	volatile bool lFwd = true;
	volatile bool rFwd = true;

	void lIntCB()
	{
		lCounter += (lFwd) ? 1 : -1;
	}

	void rIntCB()
	{
		rCounter += (rFwd) ? 1 : -1;;
	}

	void reset()
	{
		lCounter = 0;
		rCounter = 0;
	}

	double count2dist(int count)
	{
		return double(count) * ENC_STEP;
	}

	void setup()
	{
		pinMode(PIN_LEFT, INPUT);
		pinMode(PIN_RIGHT, INPUT);
		digitalWrite(PIN_LEFT, INPUT_PULLUP);
		digitalWrite(PIN_RIGHT, INPUT_PULLUP);

		attachInterrupt(digitalPinToInterrupt(PIN_LEFT), lIntCB, CHANGE);
		attachInterrupt(digitalPinToInterrupt(PIN_RIGHT), rIntCB, CHANGE);
		reset();
	}

}
