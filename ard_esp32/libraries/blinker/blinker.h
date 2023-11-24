#ifndef __BLINKER_H
#define __BLINKER_H

#include <Arduino.h>
#include <Ticker.h>

class Blinker {

	private:

		volatile int blinking_times;
		volatile int blinking_state;

	protected:

		int blink_pin;
		Ticker blinker;
		void blinking();
		static void blink_cb(Blinker *p);

	public:

		Blinker(): blinking_times(0), blinking_state(0), blink_pin(-1) {}
		void begin(int blink_pin);
		void blink(int count=1);
		void stop();

};

#endif // __BLINKER_H
