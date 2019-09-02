#include "a_turntoheading.h"
#include "motor_proc.h"
#include "hmc5883l_proc.h"

TurnToHeading::TurnToHeading(int heading) {
	this->heading = heading % 360;
	if(this->heading < 0)
		this->heading += 360;
}

bool TurnToHeading::loop() {
	if(abs(heading-headingDegrees) > 5) {
		int diff = headingDegrees - heading;
		Serial.print("diff=");
		Serial.println(diff);
		if((diff > 0 && diff < 180) || (diff < 0 && diff < -180)) {
			// turn right
			turn_right(1000);
		} else {
			// turn left
			turn_left(1000);
		}
	} else {
		stop();
		return false;
	}
	return true;
}
