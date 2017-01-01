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
				setLeftMotor(ir_power);
				setRightMotor(ir_power);
//					linVel += lvStep;
				break;
			case IR_DOWN:
				full_stopped = false;
				setLeftMotor(-ir_power);
				setRightMotor(-ir_power);
//					linVel -= lvStep;
				break;
			case IR_RIGHT:
				full_stopped = false;
				setLeftMotor(ir_power);
				setRightMotor(-ir_power);
//					angVel -= avStep;
				break;
			case IR_LEFT:
				full_stopped = false;
				setLeftMotor(-ir_power);
				setRightMotor(ir_power);
//					angVel -= avStep;
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
				setLeftMotor(ir_power * lDirection);
				setRightMotor(ir_power * rDirection);
				break;
			case IR_8:
				ir_power -= 5;
				if(ir_power < 5) {
					ir_power = 5;
				}
				setLeftMotor(ir_power * lDirection);
				setRightMotor(ir_power * rDirection);
				break;
		}
//		onVelocityRequest();
	}
}

void evFullStop(FuseID fuse, int& userData)
{
	stop(true);
//	Serial.print(millis());
//	Serial.println("Full stop");
}

void evHeadServoDetach(FuseID fuse, int& userData)
{
	if(millis() - last_head_servo_move_ts > 1000 && head_servo.attached()) {
		head_servo.detach();
	}
}
