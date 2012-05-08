#ifndef __SERVOX_H

#define __SERVOX_H

#include <Servo.h>

class ServoX: public Servo
{
	private:
		float __angle;
		float __speed; //angle per sec

		float __last_time;
	public:
		ServoX();
		void setAngle(int angle, int speed);
		void update();
};

#endif //__SERVOX_H
