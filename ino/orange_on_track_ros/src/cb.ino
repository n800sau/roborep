const unsigned long IR_UP = 0x10;
const unsigned long IR_DOWN = 0x16;
const unsigned long IR_LEFT = 0x12;
const unsigned long IR_RIGHT = 0x14;
const unsigned long IR_STOP = 0x13;
const unsigned long IR_L = 0x0C;
const unsigned long IR_ENTER = 0x0D;
const unsigned long IR_R = 0x0E;
const unsigned long IR_2 = 0x04;
const unsigned long IR_8 = 0x0A;


int ir_power = 40;

void evIRcmd(FuseID fuse, int& data)
{
	if (Serial3.available()) {
		int inByte = Serial3.read();
		switch(inByte) {
			case IR_UP:
				full_stopped = false;
				setLeftMotor(ir_power, true);
				setRightMotor(ir_power, true);
				break;
			case IR_DOWN:
				full_stopped = false;
				setLeftMotor(ir_power, false);
				setRightMotor(ir_power, false);
				break;
			case IR_RIGHT:
				full_stopped = false;
				setLeftMotor(ir_power, true);
				setRightMotor(ir_power, false);
				break;
			case IR_LEFT:
				full_stopped = false;
				setLeftMotor(ir_power, false);
				setRightMotor(ir_power, true);
				break;
			case IR_STOP:
				stop(true);
				break;
			case IR_L:
				sonarAngle = 70 + SONAR_ANGLE_MIN + (SONAR_ANGLE_MAX - SONAR_ANGLE_MIN) / 2;
				head_servo_move_to(sonarAngle);
				break;
			case IR_R:
				sonarAngle = -70 + SONAR_ANGLE_MIN + (SONAR_ANGLE_MAX - SONAR_ANGLE_MIN) / 2;
				head_servo_move_to(sonarAngle);
				break;
			case IR_ENTER:
				sonarAngle = SONAR_ANGLE_MIN + (SONAR_ANGLE_MAX - SONAR_ANGLE_MIN) / 2;
				head_servo_move_to(sonarAngle);
				break;
			case IR_2:
				ir_power += 5;
				if(ir_power > 90) {
					ir_power = 90;
				}
				if(!full_stopped) {
					setLeftMotor(lPower+5, lFwd);
					setRightMotor(rPower+5, rFwd);
				}
				break;
			case IR_8:
				ir_power -= 5;
				if(ir_power < 5) {
					ir_power = 5;
				}
				if(!full_stopped) {
					setLeftMotor(lPower-5, lFwd);
					setRightMotor(rPower-5, rFwd);
				}
				break;
		}
	}
}

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
//	Serial.print(millis());
//	Serial.println("Full stop");
}

void evStop(FuseID fuse, int& userData)
{
//	Serial.print(millis());
	stop();
}

void evBackLeftRight(FuseID fuse, int& pwr)
{
//	Serial.print(millis());
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
	head_servo_move_to(sonarAngle);
}

void evLaserScan(FuseID fuse, int &userData)
{
	if(laser_scan_allowed) {
		fill_laser_scan();
		pub_laser_scan.publish(&laser_scan_msg);
	}
}

void evPIDupdate(FuseID fuse, int &userData)
{
	if(cmd_vel_mode) {
		updatePID();
	}
}

void evHeadServoDetach(FuseID fuse, int& userData)
{
	if(millis() - last_head_servo_move_ts > 1000 && head_servo.attached()) {
		head_servo.detach();
	}
}

