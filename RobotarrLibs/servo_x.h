#ifndef __SERVOX_H

#define __SERVOX_H

#include <Servo.h>

class ServoX: public Servo
{
	private:
		int __angle;
		int __speed; //angle per sec

		unsigned long __last_time;
		bool __finished;
	public:
		ServoX();
		void setAngle(int angle, int speed=0);
		bool update();
                bool isFinished();
};

#endif //__SERVOX_H
