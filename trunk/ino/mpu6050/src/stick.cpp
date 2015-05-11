#include <Arduino.h>
#include <Servo.h>

#define SPOS_MIN 1000
#define SPOS_MAX 2000
#define SPOS_STEP 1

static Servo stickservo;
static Servo tiltservo;
static Servo panservo;

#define SERVO stickservo

int spos = 1500;
int soff = SPOS_STEP;

void stick_setup()
{
	tiltservo.attach(11);
	tiltservo.writeMicroseconds(spos);
	panservo.attach(10);
	panservo.writeMicroseconds(spos);
	stickservo.attach(9);
	stickservo.writeMicroseconds(spos);

}

void stick_loop()
{
	spos += soff;
	if(spos <= SPOS_MIN) {
		spos = SPOS_MIN + 1;
		soff = SPOS_STEP;
	}
	if(spos >= SPOS_MAX) {
		spos = SPOS_MAX - 1;
		soff = -SPOS_STEP;
	}
//	Serial.println(spos);
	SERVO.writeMicroseconds(spos);

}
