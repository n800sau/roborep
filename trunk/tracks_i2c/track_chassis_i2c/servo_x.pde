#include <Servo.h>

class ServoX: public Servo
{
	private:
		float __angle;
		float __speed; //angle per sec

		float __last_time;
	public:
		ServoX();
		void setAngle(float angle);
		void update();
};

ServoX::ServoX()
	Servo()
{
	__angle = -1;
}

//speed - degree in secs
void ServoX::setAngle(int angle, int speed)
{
	__angle = angle;
	__speed = speed;
	__last_time = millis();
}

void ServoX::update()
{
	if(__angle >= 0) {
		now = mills();
		step = __speed / 1000 * (now - __last_time);
		__last_time = now;
		int angle = read();
		bool dir = (__angle > angle)? 1 : -1;
		angle += dir * step;
		if((dir>0 && angle > __angle) || (dir<0 && angle < __angle))
			angle = __angle;
		write(angle);
		angle = -1;
	}
}
