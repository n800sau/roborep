#include <PID_v1.h>
#include <Cmd.h>

// wheel dimension
const float WL_CountPerRev = 128;
const float WL_Diameter = 0.06;
const float WL_Width = 0.02;
const float WL_Step = WL_Diameter * PI / WL_CountPerRev;

const int m1pwm = PB8;
const int in1 = PB15;
const int in2 = PB14;

const int m2pwm = PB9;
const int in3 = PB12;
const int in4 = PB13;

#define LED_BUILDING PC13

const int motor1QeiAPin	 = PB3; //external interrupt 1 (UNO) or 0 (Leonardo)
const int motor1QeiBPin	 = PB4;
const int motor2QeiAPin = PA15; //external interrupt 0 (UNO) or 1 (Leonardo)
const int motor2QeiBPin = PA12;

volatile int motor1Counts = 0, motor2Counts = 0;
volatile int oldmotor1Counts = 0, oldMmotor2Counts = 0;
// micros per encoder step
volatile unsigned long int motor1ElapsedTime = 0, motor2ElapsedTime = 0;
volatile unsigned long int motor1OldElapsedTime = 0, motor2OldElapsedTime = 0;

int motor1Power = 0;
int motor2Power = 0;

bool stop_mode = true;

//PID

//adjust these values to fit your own design
double Kp = 20;
double Kd = 10;
double Ki = 10;

double m1_vel = 0;
double m1_curvel, m1_power;
PID m1_pid(&m1_curvel, &m1_power, &m1_vel, Kp, Ki, Kd, DIRECT);

double m2_vel = 0;
double m2_curvel, m2_power;
PID m2_pid(&m2_curvel, &m2_power, &m2_vel, Kp, Ki, Kd, DIRECT);

void Motor1quickQEI()
{
	const int iv = 1;
	//a fast(ish) QEI function
	int state = 0;
	unsigned long int microTime;
	oldmotor1Counts = motor1Counts;
	state = digitalRead(motor1QeiAPin) << 1;
	state = state|digitalRead(motor1QeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			motor1Counts -= iv;
			break;
		case 1:
		case 2:
			motor1Counts += iv;
			break;
	}

	microTime = micros();
	motor1ElapsedTime = microTime-motor1OldElapsedTime;
	motor1OldElapsedTime = microTime;
}

void Motor2quickQEI()
{
	const int iv = -1;
	//a fast(ish) QEI function
	int state = 0;
	int microTime;
	state = digitalRead(motor2QeiAPin) << 1;
	state = state|digitalRead(motor2QeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			motor2Counts += iv;
			break;
		case 2:
			motor2Counts -= iv;
			break;
	}
	microTime = micros();
	motor2ElapsedTime = microTime-motor2OldElapsedTime;
	motor2OldElapsedTime = microTime;
}



void setup() {
	Serial.begin(115200);
	// Set up the built-in LED pin as an output:
	pinMode(LED_BUILDING, OUTPUT);
	pinMode(m1pwm, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(m2pwm, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	pinMode(motor1QeiAPin, INPUT_PULLUP);
	pinMode(motor2QeiAPin, INPUT_PULLUP);
	pinMode(motor1QeiBPin, INPUT_PULLUP);
	pinMode(motor2QeiBPin, INPUT_PULLUP);

	//setup left PID
	m1_pid.SetMode(AUTOMATIC);
	m1_pid.SetSampleTime(10);
	m1_pid.SetOutputLimits(-255, 255);

	//setup right PID
	m2_pid.SetMode(AUTOMATIC);
	m2_pid.SetSampleTime(10);
	m2_pid.SetOutputLimits(-255, 255);

	cmdInit(&Serial);
	cmdAdd("set", set_param);
	cmdAdd("pid", get_PID);
	cmdAdd("status", get_Status);
	cmdAdd("go", go);
	cmdAdd("stop", full_stop);

	attachInterrupt(motor1QeiAPin, Motor1quickQEI, CHANGE);
	attachInterrupt(motor2QeiAPin, Motor2quickQEI, CHANGE);
	//start the wheels
	motor1ElapsedTime = micros();
	motor2ElapsedTime = micros();
}

// pwr: -255 .. 255
void move_motor(int pin1, int pin2, int pin_pwm, int pwr)
{
	if(pwr < 0) {
		digitalWrite(pin1, HIGH);
		digitalWrite(pin2, LOW);
	} else {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, HIGH);
	}
	analogWrite(pin_pwm, pwr);
}

void move()
{
	if(stop_mode) {
		move_motor(in1, in2, m1pwm, 0);
		move_motor(in3, in4, m2pwm, 0);
	} else {
		move_motor(in1, in2, m1pwm, motor1Power);
		move_motor(in3, in4, m2pwm, motor2Power);
	}
}

void loop()
{
	digitalWrite(LED_BUILDING,!digitalRead(LED_BUILDING));// Turn the LED from off to on, or on to off
	m1_curvel = 1000000. / motor1ElapsedTime * WL_Step;
	m2_curvel = 1000000. / motor2ElapsedTime * WL_Step;

	Serial.print("M1:");
	Serial.print(m1_curvel);
	Serial.print(", M2:");
	Serial.println(m2_curvel);

	m1_pid.Compute();
	m2_pid.Compute();

	motor1Power = m1_power;
	motor2Power = m2_power;

	move();

	delay(10);

}


void display_error(String msg)
{
	Stream *s = cmdGetStream();
	s->println("{\"error\":\"" + msg + "\"}");
}

void display_ok(String msg="ok")
{
	Stream *s = cmdGetStream();
	s->println("{\"result\":\"" + msg + "\"}");
}

void display_param(String name, float value)
{
	Stream *s = cmdGetStream();
	s->println("{\"name\":\"" + name + "\",\"value\":" + value + "}");
}

void set_param(int argc, char **argv)
{
	if(argc >= 3) {
		String param = String(argv[1]);
		float value = String(argv[2]).toFloat();
		if(param == "Kp") { Kp = value; display_param("Kp", Kp); }
		else if(param == "Kd") { Kd = value; display_param("Kd", Kd); }
		else if(param == "Ki") { Ki = value; display_param("Ki", Ki); }
		else if(param == "vel") { m1_vel = m2_vel = value; display_param("vel", value); }
		else if(param == "m1vel") { m1_vel = value; display_param("m1vel", m1_vel); }
		else if(param == "m2vel") { m2_vel = value; display_param("m2vel", m2_vel); }
		else display_error("bad parameter name:" + param);
	} else {
		display_error("format");
	}
}

void get_Status(int argc, char **argv)
{
	String reply = String("{\"m1\":") +
		"{\"pwr\":" + String(motor1Power) + ",\"cnt\":" + String(motor1Counts) +
			",\"v\":" + String(m1_curvel, 8) + ",\"vp\":" + m1_vel + "},\"m2\":" +
		"{\"pwr\":" + String(motor2Power) + ",\"cnt\":" + String(motor2Counts) +
			",\"v\":" + String(m2_curvel, 8) + ",\"vp\":" + m2_vel +  "}}";
	Serial.println(reply);
}

void get_PID(int argc, char **argv)
{
	Serial.print("{\"Kp\":");
	Serial.print(Kp);
	Serial.print(",\"Kd\":");
	Serial.print(Kd);
	Serial.print(",\"Ki\":");
	Serial.print(Ki);
	Serial.println("}");
}

void full_stop(int argc, char **argv)
{
	motor1Power = 0;
	motor2Power = 0;
	m1_vel = m2_vel = 0;
	stop_mode = true;
	display_ok();
}

void go(int argc, char **argv)
{
	stop_mode = false;
	display_ok();
}
