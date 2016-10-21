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
	}
	moving_straight = false;
	powerOffset = 0;
	setLeftMotor(0, false);
	setRightMotor(0, false);
}

void straight(int pwr, bool fwd)
{
	setLeftMotor(pwr, fwd);
	setRightMotor(pwr, fwd);
	moving_straight = true;
	powerOffset = 0;
	fwd_heading = headingDegrees;
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
//		Serial.print("Right:");
//		Serial.print(pwm);
//		Serial.println((fwd) ? ", fwd": ", back");
	}
}

float getRange_Ultrasound()
{
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
	return duration / 58.2 / 100;
}
