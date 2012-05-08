#include <WProgram.h>
#include "servo_x.h"

ServoX::ServoX():Servo()
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
		int now = millis();
		float step = __speed / 1000 * (now - __last_time);
                if(step > 0) {
  		  __last_time = now;
		  int angle = read();
//                  Serial.print("read:");
//                  Serial.print(angle);
		  bool dir = (__angle > angle)? 1 : -1;
		  angle += dir * step;
//                  Serial.print(", step:");
//                  Serial.print(step);
		  if((dir>0 && angle > __angle) || (dir<0 && angle < __angle))
			angle = __angle;
//                  Serial.print(",write:");
//                  Serial.println(angle);
		  write(angle);
                }
	}
}

