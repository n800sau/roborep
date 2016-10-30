// straight move deviation fix
void evFixDir(FuseID fuse, int& data)
{
	const float Kp = 0.3;
	if(moving_straight && !full_stopped) {
		int aoff = angle_offset(fwd_heading, headingDegrees);
		if((aoff < 0 && lFwd && rFwd) || (aoff > 0 && (!lFwd) && (!rFwd))) {
			powerOffset = Kp * abs(aoff);
			// left more right less
		} else {
			powerOffset = -Kp * abs(aoff);
			// left less right more
		}
		setLeftMotor(lPower, lFwd);
		setRightMotor(rPower, rFwd);
//		Serial.println(aoff);
	}
}

void evFullStop(FuseID fuse, int& userData)
{
	stop(true);
	Serial.print(millis());
	Serial.println("Full stop");
}

void evStop(FuseID fuse, int& userData)
{
	Serial.print(millis());
	stop();
}

void evLeftRight(FuseID fuse, int& pwr)
{
	Serial.print(millis());
	if(random(1)) {
		setLeftMotor(pwr, false);
		setRightMotor(pwr, true);
	} else {
		setLeftMotor(pwr, true);
		setRightMotor(pwr, false);
	}
	EventFuse::newFuse(pwr, 300, 1, evForward);
}

void evForward(FuseID fuse, int& pwr)
{
	straight(pwr, true);
}

void evChangePower(FuseID fuse, int& userData)
{
	int v = 10 * (random(1)) ? -1 : 1;
	int lPwr = lPower, rPwr = rPower;
	static int ldir = random(1);
	if(ldir>0) {
		lPwr += v;
	} else {
		rPwr += v;
	}
	ldir = !ldir;
	setLeftMotor(max(0, min(lPwr, 100)), lFwd);
	setRightMotor(max(0, min(rPwr, 100)), rFwd);
	if(!full_stopped) {
		EventFuse::newFuse(500, 1, evChangePower);
	}
}

void evSonar(FuseID fuse, int& userData)
{
	float distance = getRange_Ultrasound();
	if(distance > 0 && distance < MAX_STOP_DIST && lFwd && rFwd and (lPower > 0 || rPower > 0)) {
//		Serial.print(millis());
//		Serial.println("Too close");
		EventFuse::newFuse((lPower+rPower)/2, 500, 1, evLeftRight);
//		stop();
	}
}

