#include <blinker.h>

void Blinker::begin(int blink_pin)
{
	this->blink_pin = blink_pin;
	pinMode(this->blink_pin, OUTPUT);
	blinker.attach(0.3, Blinker::blink_cb, this);
}

void Blinker::blink_cb(Blinker *p) {
	p->blinking();
}

// attention! delays in tickers crash
void Blinker::blinking()
{
	if(blinking_times > 0) {
		if(blinking_state) {
//			Serial.println("led off");
			digitalWrite(blink_pin, LOW);
			blinking_times--;
		} else {
//			Serial.println("led on");
			digitalWrite(blink_pin, HIGH);
		}
		blinking_state = !blinking_state;
	}
}

void Blinker::blink(int count)
{
	blinking_times = count;
	blinking_state = 0;
}

void Blinker::stop()
{
	blinking_times = 0;
	blinking_state = 0;
}
