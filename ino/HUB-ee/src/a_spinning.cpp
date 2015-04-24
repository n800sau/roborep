#include "a_spinning.h"
#include "motor_proc.h"
#include "hmc5883l_proc.h"

Spinning::Spinning() {
	turn_left(10000);
	heading = headingDegrees;
	stage = 1;
}

bool Spinning::loop() {
	if(abs(heading-headingDegrees) > 5) {
		int h = heading;
		if(abs(h - headingDegrees) > 180) {
			h += 360;
		}
		if(abs(h - headingDegrees) > 90) {
			heading = headingDegrees;
			stage++;
		}
		if(stage > 4) {
			stop();
			return false;
		}
	}
	return true;
}
