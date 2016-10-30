void walk_around(int pwr, int timeout) {
	stop(true);
	full_stopped = false;
//	Serial.print(millis());
//	Serial.println("Start walking");
	straight(pwr, true);
	if(timeout <= 0) {
		timeout = 60;
	}
	EventFuse::newFuse(timeout * 500, 1, evFullStop);
}

void move2release(int pwr, bool fwd, int timeout) {
	stop(true);
	full_stopped = false;
//	Serial.print(millis());
//	Serial.println("Start releasing");
	setLeftMotor(pwr, fwd);
	setRightMotor(pwr, fwd);
	EventFuse::newFuse(500, 1, evChangePower);
	if(timeout <= 0) {
		timeout = 30;
	}
	EventFuse::newFuse(timeout * 500, 1, evFullStop);
}

// timeout in halve seconds
void stop_after(int timeout) {
	if(timeout > 0) {
		EventFuse::newFuse(timeout * 500, 1, evFullStop);
	}
}

