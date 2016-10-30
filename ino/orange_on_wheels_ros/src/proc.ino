void move2release(int pwr, bool fwd)
{
	full_stopped = false;
//	Serial.print(millis());
//	Serial.println("Start releasing");
	setLeftMotor(pwr, fwd);
	setRightMotor(pwr, fwd);
	EventFuse::newFuse(500, 1, evChangePower);
}

// timeout in halve seconds
void stop_after(int timeout)
{
	if(timeout < 0) {
		timeout = 60;
	} else {
		EventFuse::newFuse(timeout * 500, 1, evFullStop);
	}
}

void straight(int pwr, bool fwd)
{
	moving_straight = true;
	powerOffset = 0;
	fwd_heading = headingDegrees;
	setLeftMotor(pwr, fwd);
	setRightMotor(pwr, fwd);
}

