void onVelocityRequest()
{
	float left, right;
	float x = linVel; // m/s
	float th = angVel; // rad/s
	full_stopped = false;
	if(x == 0) {
		// Turn in place
		right = th * WHEEL_TRACK  * GEAR_REDUCTION / 2.0;
		left = -right;
	} else if(th == 0) {
		// Pure forward/backward motion
		left = right = x;
	} else {
		// Rotation about a point in space
		left = x - th * WHEEL_TRACK * GEAR_REDUCTION / 2;
		right = x + th * WHEEL_TRACK  * GEAR_REDUCTION / 2.0;
	}
	leftPID.TargetTicksPerFrame = round(left * TICKS_PER_METER / PID_RATE);
	rightPID.TargetTicksPerFrame = round(right * TICKS_PER_METER / PID_RATE);
	setLeftMotor(30 * sign(left));
	setRightMotor(30 * sign(right));
}
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
	setLeftMotor(0);
	setRightMotor(0);
}

void setLeftMotor(int power)
{
	if(full_stopped) {
		power = 0;
	}
	if(power == 0) {
		digitalWrite(LEFT_MOTOR_1, LOW);
		digitalWrite(LEFT_MOTOR_2, LOW);
		Serial.println("Left stopped");
	} else {
		lDirection = sign(power);
		int pwm = power2pwm(abs(power));
		if(lDirection > 0) {
			analogWrite(LEFT_MOTOR_1, pwm);
			digitalWrite(LEFT_MOTOR_2, LOW);
		} else {
			analogWrite(LEFT_MOTOR_1, pwm);
			digitalWrite(LEFT_MOTOR_2, HIGH);
		}
		Serial.print("Left:");
		Serial.print(pwm);
		Serial.println((lDirection > 0) ? ", fwd": ", back");
	}
}

void setRightMotor(int power)
{
	if(full_stopped) {
		power = 0;
	}
	if(power == 0) {
		digitalWrite(RIGHT_MOTOR_1, LOW);
		digitalWrite(RIGHT_MOTOR_2, LOW);
		Serial.println("Right stopped");
	} else {
		rDirection = sign(power);
		int pwm = power2pwm(abs(power));
		if(rDirection > 0) {
			analogWrite(RIGHT_MOTOR_1, pwm);
			digitalWrite(RIGHT_MOTOR_2, LOW);
		} else {
			analogWrite(RIGHT_MOTOR_1, pwm);
			digitalWrite(RIGHT_MOTOR_2, HIGH);
		}
		Serial.print("Right:");
		Serial.print(pwm);
		Serial.println((rDirection > 0) ? ", fwd": ", back");
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

// timeout in halve seconds
void stop_after(int timeout)
{
	if(timeout < 0) {
		timeout = 60;
	} else {
		EventFuse::newFuse(timeout * 500, 1, evFullStop);
	}
}

void head_servo_move_to(int pos)
{
	if(!head_servo.attached()) {
		head_servo.attach(headServoPin);
	}
	if(pos < SONAR_ANGLE_MIN) pos = SONAR_ANGLE_MIN;
	if(pos > SONAR_ANGLE_MAX) pos = SONAR_ANGLE_MAX;
	head_servo.write(pos + SONAR_CENTER_OFFSET);
	last_head_servo_move_ts = millis();
}
