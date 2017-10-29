// motor pins
const int LEFT_MOTOR_FWD = 6;
const int LEFT_MOTOR_POWER = 5;

const int RIGHT_MOTOR_FWD = 9;
const int RIGHT_MOTOR_POWER = 10;

const int Eleft = 2;
const int Eright = 3;

volatile int lCounter = 0;
volatile int rCounter = 0;

void lIntCB()
{
	lCounter += 1;
}

void rIntCB()
{
	rCounter += 1;
}


void setup()
{
	Serial.begin(115200);

	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);
	digitalWrite(Eleft, INPUT_PULLUP);
	digitalWrite(Eright, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(Eleft), lIntCB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Eright), rIntCB, CHANGE);

	pinMode(LEFT_MOTOR_POWER, OUTPUT);
	pinMode(LEFT_MOTOR_FWD, OUTPUT);
	pinMode(RIGHT_MOTOR_POWER, OUTPUT);
	pinMode(RIGHT_MOTOR_FWD, OUTPUT);

	delay(20000);

	int dir = HIGH;

	analogWrite(LEFT_MOTOR_POWER, 100);
	digitalWrite(LEFT_MOTOR_FWD, dir);
	analogWrite(RIGHT_MOTOR_POWER, 100);
	digitalWrite(RIGHT_MOTOR_FWD, dir);

}

void loop()
{
	delay(500);
	Serial.print(lCounter);
	Serial.print(", ");
	Serial.println(rCounter);
	if(lCounter >= 300 || rCounter >= 300) {
		digitalWrite(LEFT_MOTOR_POWER, LOW);
		digitalWrite(LEFT_MOTOR_FWD, LOW);
		digitalWrite(RIGHT_MOTOR_POWER, LOW);
		digitalWrite(RIGHT_MOTOR_FWD, LOW);
		delay(60000);
	}
}
