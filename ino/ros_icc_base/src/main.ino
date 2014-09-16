int EN1 = 6;
int EN2 = 5;
int IN1 = 7;
int IN2 = 4;

volatile bool lMotorReverse = false;
volatile bool lMotorStop = true;
volatile bool rMotorReverse = false;
volatile bool rMotorStop = true;

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
	if(lMotorStop || micros() - intLtime < threshold )
		return;
	intLhistory = intLsignal;
	intLsignal = bitRead(PIND,2);
	if ( intLhistory==intLsignal )
		return;
	intLtime = micros();
	lEncSteps += (lMotorReverse) ? 1 : -1;
}

void rIntCB()
{
	if (rMotorStop || micros() - intRtime < threshold )
		return;
	intRhistory = intRsignal;
	intRsignal = bitRead(PIND,3);
	if ( intRhistory==intRsignal )
		return;
	intRtime = micros();
	rEncSteps += (rMotorReverse) ? 1 : -1;
}


//Latest version use pin 4 instead of pin 8
void setLeftMotor(int pwm, boolean reverse)
{
	analogWrite(EN1,pwm); //set pwm control, 0 for stop, and 255 for maximum speed
	digitalWrite(IN1,(reverse) ? HIGH : LOW);
	lMotorReverse = reverse;
	lMotorStop = pwm == 0;
}

void setRightMotor(int pwm, boolean reverse)
{
	analogWrite(EN2,pwm);
	digitalWrite(IN2,(reverse) ? HIGH : LOW);
	rMotorReverse = reverse;
	rMotorStop = pwm == 0;
}

void printEncoders()
{
	Serial.print(lEncSteps);
	Serial.print(" ");
	Serial.println(rEncSteps);
}

void setup()
{
	Serial.begin(57600);
	pinMode(EN1, OUTPUT);
	pinMode(EN2, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);
	digitalWrite(Eleft, HIGH);
	digitalWrite(Eright, HIGH);
	attachInterrupt(rInt, rIntCB, CHANGE);
	attachInterrupt(lInt, lIntCB, CHANGE);
}

void stop()
{
	setLeftMotor(0,false);
	setRightMotor(0,false);
}

void loop()
{
	char val;
	while(1)
	{
		printEncoders();
		val = Serial.read();
		if(val!=-1)
		{
			Serial.println(val);
			switch(val)
			{
				case 'w'://Move ahead
					setLeftMotor(240,true);
					setRightMotor(240,true);
					delay(100);
					stop();
					break;
				case 'x'://move back
					setLeftMotor(240,false);
					setRightMotor(240,false);
					delay(100);
					stop();
					break;
				case 'd'://turn left
					setLeftMotor(240,false);
					setRightMotor(240,true);
					delay(100);
					stop();
					break;
				case 'a'://turn right
					setLeftMotor(240,true);
					setRightMotor(240,false);
					delay(100);
					stop();
					break;
				case 's'://stop
					stop();
					break;
			}
		}
	}
}
