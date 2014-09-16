int period2stop = 200;


int lEN = 6;
int rEN = 5;
int lIN = 7;
int rIN = 4;

volatile int lMotorDest = 0;
volatile bool lMotorReverse = false;
volatile int rMotorDest = 0;
volatile bool rMotorReverse = false;

int Eleft = 3;
int Eright = 2;

int lInt = Eleft - 1;
int rInt = Eright - 1;

// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. 
//
// The threshold value I'm using limits it to 100 half pulses a second
//
// My encoder has 12 pulses per 360deg rotation and the specs say
// it is rated at a maximum of 100rpm.
//
// This threshold will permit my encoder to reach 250rpm so if it was connected
// to a motor instead of a manually operated knob I
// might possibly need to adjust it to 25000. However, this threshold
// value is working perfectly for my situation
//
volatile unsigned long threshold = 10000;


volatile long lEncSteps = 0;
volatile long rEncSteps = 0;


// Working variables for the interrupt routines
//
volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;
volatile uint8_t intLsignal = 0;
volatile uint8_t intRsignal = 0;
volatile uint8_t intLhistory = 0;
volatile uint8_t intRhistory = 0;

void lIntCB()
{
	if( micros() - intLtime < threshold )
		return;
	intLhistory = intLsignal;
	intLsignal = bitRead(PIND,2);
	if ( intLhistory==intLsignal )
		return;
	intLtime = micros();
	if(lMotorDest) {
		lMotorDest += (lMotorDest < 0) ? 1 : -1;
	}
	lEncSteps += (lMotorReverse) ? -1 : 1;
}

void rIntCB()
{
	if (micros() - intRtime < threshold )
		return;
	intRhistory = intRsignal;
	intRsignal = bitRead(PIND,3);
	if ( intRhistory==intRsignal )
		return;
	intRtime = micros();
	if(rMotorDest) {
		rMotorDest += (rMotorDest < 0) ? 1 : -1;
	}
	rEncSteps += (rMotorReverse) ? -1 : 1;
}


void setLeftMotor(int pwm, int steps)
{
	if(steps) {
		lMotorDest += steps;
	} else {
		lMotorDest = 0;
	}
	analogWrite(lEN,pwm); //set pwm control, 0 for stop, and 255 for maximum speed
	lMotorReverse = lMotorDest < 0;
	digitalWrite(lIN,(lMotorReverse) ? HIGH : LOW);
}

void setRightMotor(int pwm, int steps)
{
	if(steps) {
		rMotorDest += steps;
	} else {
		rMotorDest = 0;
	}
	analogWrite(rEN,pwm);
	rMotorReverse = rMotorDest < 0;
	digitalWrite(rIN,(rMotorReverse < 0) ? HIGH : LOW);
}

void stop()
{
	setLeftMotor(0, 0);
	setRightMotor(0, 0);
}

int lastLVal = -1;
int lastRVal = -1;

void printDests()
{
	Serial.print("M ");
	Serial.print(lMotorDest);
	Serial.print(" ");
	Serial.println(rMotorDest);
}

void printEncoders()
{
	int lval = lEncSteps;
	int rval = rEncSteps;
	if(lastLVal != lval || lastRVal != rval) {
		Serial.print(lastLVal=lval);
		Serial.print(" ");
		Serial.println(lastRVal=rval);
	}
}

void setup()
{
	Serial.begin(57600);
	pinMode(lEN, OUTPUT);
	pinMode(rEN, OUTPUT);
	pinMode(lIN, OUTPUT);
	pinMode(rIN, OUTPUT);
	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);
	digitalWrite(Eleft, HIGH);
	digitalWrite(Eright, HIGH);
	attachInterrupt(rInt, rIntCB, CHANGE);
	attachInterrupt(lInt, lIntCB, CHANGE);
}

void loop()
{
	char val;
	while(1)
	{
		printEncoders();
		if(lMotorDest == 0) {
			setLeftMotor(0, 0);
		}
		if(rMotorDest == 0) {
			setRightMotor(0, 0);
		}
		val = Serial.read();
		if(val!=-1)
		{
			Serial.println(val);
			switch(val)
			{
				case 'w'://Move ahead
					setLeftMotor(240,5);
					setRightMotor(240,5);
					printDests();
					delay(period2stop);
					printDests();
					stop();
					break;
				case 'x'://move back
					setLeftMotor(240,-5);
					setRightMotor(240,-5);
					printDests();
					delay(period2stop);
					printDests();
					stop();
					break;
				case 'd'://turn left
					setLeftMotor(240,-5);
					setRightMotor(240,5);
					printDests();
					delay(period2stop);
					printDests();
					stop();
					break;
				case 'a'://turn right
					setLeftMotor(240,5);
					setRightMotor(240,-5);
					printDests();
					delay(period2stop);
					printDests();
					stop();
					break;
				case 's'://stop
					stop();
					break;
				case 'p':
					period2stop += 100;
					Serial.println(period2stop);
					break;
				case 'l':
					period2stop -= 100;
					Serial.println(period2stop);
					break;
			}
		}
	}
}
