float count2dist(int count)
{
	return count * ENC_STEP;
}

void resetCounters()
{
	lCounter = rCounter = 0;
}

int power2pwm(int power)
{
	int rs = MIN_PWM + (MAX_PWM-MIN_PWM) / 100. * power;
	if(rs < MIN_PWM) {
		rs = MIN_PWM;
	} else if(rs > MAX_PWM) {
		rs = MAX_PWM;
	}
	return rs;
}

void stop(bool full)
{
	if(full) {
		full_stopped = true;
		cmd_vel_mode = false;
		strncpy(current_command, "", sizeof(current_command));
	}
	moving_straight = false;
	powerOffset = 0;
	setLeftMotor(0, false);
	setRightMotor(0, false);
}

void setLeftMotor(int power, bool fwd)
{
	if(full_stopped) {
		power = 0;
	}
	lPower = power;
	if(moving_straight && !full_stopped) {
		power += powerOffset;
	}
	lFwd = fwd;
	if(power == 0) {
		digitalWrite(LEFT_MOTOR_1, LOW);
		digitalWrite(LEFT_MOTOR_2, LOW);
//		Serial.println("Left stopped");
	} else {
		int pwm = power2pwm(power);
		if(fwd) {
			analogWrite(LEFT_MOTOR_1, pwm);
			digitalWrite(LEFT_MOTOR_2, LOW);
		} else {
			analogWrite(LEFT_MOTOR_2, pwm);
			digitalWrite(LEFT_MOTOR_1, LOW);
		}
//		nh.loginfo(("Left power:" + String(pwm) + ", fwd:" + String(fwd)).c_str());
//		Serial.print("Left:");
//		Serial.print(pwm);
//		Serial.println((fwd) ? ", fwd": ", back");
	}
}

void setRightMotor(int power, bool fwd)
{
	if(full_stopped) {
		power = 0;
	}
	rPower = power;
	if(moving_straight && !full_stopped) {
		power -= powerOffset;
	}
	rFwd = fwd;
	if(power == 0) {
		digitalWrite(RIGHT_MOTOR_1, LOW);
		digitalWrite(RIGHT_MOTOR_2, LOW);
//		Serial.println("Right stopped");
	} else {
		int pwm = power2pwm(power);
		if(fwd) {
			analogWrite(RIGHT_MOTOR_1, pwm);
			digitalWrite(RIGHT_MOTOR_2, LOW);
		} else {
			analogWrite(RIGHT_MOTOR_2, pwm);
			digitalWrite(RIGHT_MOTOR_1, LOW);
		}
//		nh.loginfo(("Right power:" + String(pwm) + ", fwd:" + String(fwd)).c_str());
//		Serial.print("Right:");
//		Serial.print(pwm);
//		Serial.println((fwd) ? ", fwd": ", back");
	}
}

float getRange_Ultrasound(int attempts, int trigPin, int echoPin)
{
	float rs = 0;
	int count = 0, i;
	float dist = MAX_RANGE + 1;
	for(i=0; i<min(attempts, 1); i++) {
		if(i > 0) {
			// to prevent aftershock reaction
			delayMicroseconds(10);
		}
		long duration = -1; // Duration used to calculate distance
//	noInterrupts();
		// The following trigPin/echoPin cycle is used to determine the
		// distance of the nearest object by bouncing soundwaves off of it.
		digitalWrite(trigPin, LOW);
		delayMicroseconds(2);

		digitalWrite(trigPin, HIGH);
		delayMicroseconds(10);

		digitalWrite(trigPin, LOW);
//	interrupts();
		duration = pulseIn(echoPin, HIGH);

		//Calculate the distance (in m) based on the speed of sound.
		dist = duration / 58.2 / 100;
		if(dist >= MIN_RANGE && dist <= MAX_RANGE) {
			rs += dist;
			count++;
		}
	}
	if(count > 0) {
		rs = rs / count;
	} else {
		rs = dist;
	}
	return rs;
}

float getRange_BackUltrasound(int attempts)
{
	return getRange_Ultrasound(attempts, backTrigPin, backEchoPin);
}

float getRange_HeadUltrasound(int attempts)
{
	return getRange_Ultrasound(attempts, headTrigPin, headEchoPin);
}
