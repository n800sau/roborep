#include <Arduino.h>
#include "servo_x.h"

ServoX::ServoX():Servo()
{
	__angle = -1;
	__speed = 1;
	__finished = true;
}

//speed - degree in secs
void ServoX::setAngle(int angle, int speed)
{
	if(angle >= 0) {
		__angle = angle;
		if (speed > 0)
			__speed = speed;
		__last_time = millis();
		__finished = false;
	}
}

void ServoX::stop()
{
	int angle = read();
	if(angle != __angle) {
		setAngle(angle);
		__finished = true;
	}
}

bool ServoX::update()
{
	if (__angle >= 0 && !__finished) {
		unsigned long now = millis();
//		Serial.print("diff:");
//		Serial.print(now-__last_time);
		float step = __speed / 1000. * (now - __last_time);
//		Serial.print(",step:");
//		Serial.println(step);
		if (step >= 1) {
			__last_time = now;
			int angle = read();
			if(angle != __angle) {
				int dir = (__angle > angle) ? 1 : -1;
				angle += dir * step;
				if ((dir > 0 && angle > __angle) || (dir < 0 && angle < __angle))
					angle = __angle;
				write(angle);
			}
			__finished = angle == __angle;
		}
	}
//			Serial.print("finished:");
//			Serial.println(__finished);
	return __finished;
}

bool ServoX::isFinished()
{
	return __finished;
}
