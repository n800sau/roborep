#include <PID_v1.h>
#include <Cmd.h>
#include <EventFuse.h>

const int m1pwm = PB9;
const int m1_in3 = PB12;
const int m1_in4 = PB13;

const int m2pwm = PB8;
const int m2_in1 = PB15;
const int m2_in2 = PB14;


#define LED_BUILDING PC13

const int motor1QeiAPin = PA15;
const int motor1QeiBPin = PA12;

const int motor2QeiAPin	 = PB3;
const int motor2QeiBPin	 = PB4;


const int minPwr = 128;
const int maxPwr = 255;
volatile int motor1Counts = 0, motor2Counts = 0;
volatile int oldMotor1Counts = 0, oldMotor2Counts = 0;
// micros per encoder step
volatile unsigned long int motor1ElapsedTime = 0, motor2ElapsedTime = 0;
volatile unsigned long int motor1OldElapsedTime = 0, motor2OldElapsedTime = 0;

bool cur_m1_fwd = true;
bool cur_m2_fwd = true;

int motor1Power = 0;
int motor2Power = 0;

float m1req_vel = 0;
float m2req_vel = 0;


bool stop_mode = true;

//PID

//adjust these values to fit your own design
double Kp = 20;
double Kd = 3;
double Ki = 10;

bool m1_fwd = true;
double m1_vel = 0;
double m1_curvel, m1_power;
PID m1_pid(&m1_curvel, &m1_power, &m1_vel, Kp, Ki, Kd, DIRECT);

bool m2_fwd = true;
double m2_vel = 0;
double m2_curvel, m2_power;
PID m2_pid(&m2_curvel, &m2_power, &m2_vel, Kp, Ki, Kd, DIRECT);

bool st_changed = false;

void Motor1quickQEI()
{
	const int iv = 1;
	//a fast(ish) QEI function
	int state = 0;
	unsigned long int microTime;
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
	cur_m1_fwd = oldMotor1Counts - motor1Counts;
	oldMotor1Counts = motor1Counts;
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
	cur_m2_fwd = oldMotor1Counts - motor1Counts;
	oldMotor2Counts = motor2Counts;
}

void setup() {
	Serial.begin(115200);
	// Set up the built-in LED pin as an output:
	pinMode(LED_BUILDING, OUTPUT);
	pinMode(m1pwm, OUTPUT);
	pinMode(m2_in1, OUTPUT);
	pinMode(m2_in2, OUTPUT);
	pinMode(m2pwm, OUTPUT);
	pinMode(m1_in3, OUTPUT);
	pinMode(m1_in4, OUTPUT);
	pinMode(motor1QeiAPin, INPUT_PULLUP);
	pinMode(motor2QeiAPin, INPUT_PULLUP);
	pinMode(motor1QeiBPin, INPUT_PULLUP);
	pinMode(motor2QeiBPin, INPUT_PULLUP);

	//setup left PID
	m1_pid.SetMode(AUTOMATIC);
	m1_pid.SetSampleTime(10);
	m1_pid.SetOutputLimits(minPwr, maxPwr);

	//setup right PID
	m2_pid.SetMode(AUTOMATIC);
	m2_pid.SetSampleTime(10);
	m2_pid.SetOutputLimits(minPwr, maxPwr);

	cmdInit(&Serial);
	cmdAdd("set", set_param);
	cmdAdd("pid", get_PID);
	cmdAdd("stat", get_Status);
	cmdAdd("g", go);
	cmdAdd("s", full_stop);

	attachInterrupt(motor1QeiAPin, Motor1quickQEI, CHANGE);
	attachInterrupt(motor2QeiAPin, Motor2quickQEI, CHANGE);
	//start the wheels
	motor1ElapsedTime = micros();
	motor2ElapsedTime = micros();

	EventFuse::newFuse(20000, INF_REPEAT, OutputEvent );
	EventFuse::newFuse(5000, INF_REPEAT, LedEvent );

	Serial.println("READY");
}

// pwr: -255 .. 255
void move_motor(int pin1, int pin2, int pin_pwm, int pwr, bool fwd)
{
	if(fwd) {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, HIGH);
	} else {
		digitalWrite(pin1, HIGH);
		digitalWrite(pin2, LOW);
	}
	analogWrite(pin_pwm, pwr);
}

void move()
{
	if(stop_mode) {
		move_motor(m2_in1, m2_in2, m1pwm, 0, true);
		move_motor(m1_in3, m1_in4, m2pwm, 0, true);
	} else {
		move_motor(m2_in1, m2_in2, m1pwm, motor1Power, m1_fwd);
		move_motor(m1_in3, m1_in4, m2pwm, motor2Power, m2_fwd);
	}
}

void OutputEvent(FuseID fuse, int& userData)
{
	if(st_changed) {
		OutputStatus();
		st_changed = false;
	}
}

void OutputStatus()
{
		Serial.println("*******************************");

		Serial.print("V1:");
		Serial.print(m1_vel);
		Serial.print(",V2:");
		Serial.println(m2_vel);

		Serial.print("CV1:");
		Serial.print(m1_curvel);
		Serial.print(",CV2:");
		Serial.println(m2_curvel);

		Serial.print("M1 pwr:");
		Serial.print(m1_power);
		Serial.print(",M2 pwr:");
		Serial.println(m2_power);

		Serial.println("*******************************");

}

void LedEvent(FuseID fuse, int& userData)
{
	digitalWrite(LED_BUILDING,!digitalRead(LED_BUILDING));// Turn the LED from off to on, or on to off
}

void loop()
{
	cmdPoll();
	if(m1req_vel == 0 && m2req_vel == 0) {
		stop_mode = true;
	}
	if(stop_mode) {
		move();
	} else {

		m1_fwd = m1req_vel >= 0;
		m2_fwd = m2req_vel >= 0;
		m1_vel = fabs(m1req_vel);
		m2_vel = fabs(m2req_vel);

		m1_curvel = motor1ElapsedTime > 0 ? ((cur_m1_fwd ? 1000000. : -1000000.) / motor1ElapsedTime) : 0;
		m2_curvel = motor2ElapsedTime > 0 ? ((cur_m2_fwd ? 1000000. : -1000000.) / motor2ElapsedTime) : 0;

		if(m1_vel == 0) {
			st_changed = motor1Power != 0;
			motor1Power = 0;
		} else if(m1_pid.Compute()) { 
			st_changed = true;
			motor1Power = m1_power;
		}

		if(m2_vel == 0) {
			st_changed = motor2Power != 0;
			motor2Power = 0;
		} else if(m2_pid.Compute()) {
			st_changed = true;
			motor2Power = m2_power;
		}
		if(st_changed) {
			move();
		}

	}

	EventFuse::burn();
	delayMicroseconds(100);

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
		else if(param == "vel") { m1req_vel = m2req_vel = value; display_param("vel", value); }
		else if(param == "m1vel") { m1req_vel = value; display_param("m1vel", m1req_vel); }
		else if(param == "m2vel") { m2req_vel = value; display_param("m2vel", m2req_vel); }
		else display_error("bad parameter name:" + param);
	} else {
		display_error("format");
	}
}

void get_Status(int argc, char **argv)
{
	String reply = String("{\"m1\":") +
		"{\"pwr\":" + String(motor1Power) + ",\"cnt\":" + String(motor1Counts, 3) +
			",\"v\":" + String(m1_curvel, 8) + ",\"vp\":" + m1req_vel + "},\"m2\":" +
		"{\"pwr\":" + String(motor2Power) + ",\"cnt\":" + String(motor2Counts, 3) +
			",\"v\":" + String(m2_curvel, 8) + ",\"vp\":" + m2req_vel +  "}}";
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
	m1req_vel = m2req_vel = 0;
	stop_mode = true;
	Stream *s = cmdGetStream();
	s->println("{\"result\":\"stopped\"}");
}

void go(int argc, char **argv)
{
	stop_mode = false;
	display_ok();
}
