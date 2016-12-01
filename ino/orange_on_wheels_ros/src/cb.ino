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

void evBackLeftRight(FuseID fuse, int& pwr)
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

// to stop movement in case of obstacle
void evHeadSonar(FuseID fuse, int& userData)
{
	float distance = getRange_HeadUltrasound();
	if(distance > 0 && distance < MAX_STOP_DIST && lFwd && rFwd && (lPower > 0 || rPower > 0) && sonarAngle == 90) {
//		Serial.print(millis());
//		Serial.println("Too close");
		EventFuse::newFuse((lPower+rPower)/2, 500, 1, evBackLeftRight);
//		stop();
	}
}

// to stop movement in case of obstacle
void evBackSonar(FuseID fuse, int& userData)
{
	float distance = getRange_BackUltrasound();
	if(distance > 0 && distance < MAX_STOP_DIST && !lFwd && !rFwd && (lPower > 0 || rPower > 0)) {
//		Serial.print(millis());
//		Serial.println("Too close");
		EventFuse::newFuse(500, 1, evStop);
//		stop();
	}
}

void evMoveSonar(FuseID fuse, int &userData)
{
	sonarAngle += sonarIncr;
	if(sonarAngle >= 180) {
		sonarIncr = -SONAR_INCR;
	} else if(sonarAngle < 20) {
		sonarIncr = SONAR_INCR;
	}
	head_servo.attach(headServoPin);
	head_servo.write(sonarAngle);
//	head_servo.detach();
}

void evLaserScan(FuseID fuse, int &userData)
{
	if(laser_scan_allowed) {
		fill_laser_scan();
		pub_laser_scan.publish(&laser_scan_msg);
	}
}
