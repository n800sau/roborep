#include <VNH3SP30.h>
#include <Wire.h>
#include <Ticker.h>

const byte I2C_SLAVE_ADDRESS = 8;
const byte I2C_REG_SET_TEMP = 1;
const byte I2C_REG_READ_PWM = 0x10;
const byte I2C_REG_READ_TEMP = 0x11;

// peltier H-bridge
VNH3SP30 Peltier;

// peltier H-bridge pins
#define M1_PWM 3    // pwm pin
#define M1_INA 4    // control pin INA
#define M1_INB 5    // control pin INB
#define M1_DIAG 6   // diagnose pins (combined DIAGA/ENA and DIAGB/ENB)
#define M1_CS A0    // current sense pin

#define USE_PID

#ifdef USE_PID

#include <PID_v1.h>

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double hKp=150, hKi=1, hKd=100;
double cKp=250, cKi=0, cKd=100;
PID heaterPID(&Input, &Output, &Setpoint, hKp, hKi, hKd, REVERSE);

#endif //USE_PID

int heater_pwm = 0;
const int MIN_PWM = -400, MAX_PWM = 400;

// NTC
const int TEMP_PIN = A2;

// Fan
const int FAN_PIN = 7;

#define UNKNOWN_VAL -1000

double temp = UNKNOWN_VAL, temp2set = UNKNOWN_VAL;
const uint16_t max_temp = 100, min_temp = 0;

float Vcc = 3.3;
float Vref = 1.1;
const float T_0 = 273.15;
const float T_25 = T_0 + 25;
// 100k NTC
const float beta = 3950;
const float R_25 = 100000L; // 100k ohm
const unsigned long Rs = 470000L;

void display_status()
{
	if(temp != UNKNOWN_VAL) {
		Serial.print(F("T "));
		Serial.println(temp);
	}
	if(temp2set != UNKNOWN_VAL) {
		Serial.print(F("> "));
		Serial.println(temp2set);
	}
	Serial.print(F("PWM:"));
	Serial.println(heater_pwm);
}

int tempAnalogRead()
{
	int val = 0;
	for(int i = 0; i < 20; i++) {
		val += analogRead(TEMP_PIN);
		delay(1);
	}
	val = val / 20;
	return val;
}

double read_temp(int pin)
{
	double v, r;
//	Serial.print(F("TEMP_PIN value:"));
//	Serial.println(analogRead(pin));
	v = Vref*tempAnalogRead()/1024.;
	r = v/((Vcc-v)/Rs);
//	Serial.print(F("R:"));
//	Serial.println(r);
//	Serial.print(F("V:"));
//	Serial.println(v);
// R = R_25 * Math.exp(beta * ((1 / (T + T_0)) - (1 / T_25)));
	return 1 / ((log(r / R_25) / beta) + 1/T_25) - T_0;
}

void update_temp()
{
	temp = read_temp(TEMP_PIN);
	if(temp2set == UNKNOWN_VAL) {
		temp2set = min(max(temp, min_temp), max_temp);
		Serial.println(F("Ready"));
	}
}

void set_temp2set(int temp)
{
#ifdef USE_PID
	if(temp2set < temp) {
		// K for cooling
		heaterPID.SetTunings(cKp, cKi, cKd);
	} else {
		// K for heating
		heaterPID.SetTunings(hKp, hKi, hKd);
	}
#endif // USE_PID
	temp2set = temp;
}

void process_proc()
{
	update_temp();
#ifdef USE_PID
	Input = temp;
	Setpoint = temp2set;
//	Serial.print("Error:");
//	Serial.println(temp2set-temp);
	heaterPID.Compute();
	heater_pwm = Output;
#else
	if(temp < temp2set) {
		heater_pwm = MAX_PWM;
	} else if(temp > temp2set) {
		heater_pwm = MIN_PWM;
	}
#endif // USE_PID
	Peltier.setSpeed(heater_pwm); // motor full-speed "backward"
	if(heater_pwm > MAX_PWM/2) {
		digitalWrite(FAN_PIN, LOW);
	} else if(heater_pwm < 0) {
		digitalWrite(FAN_PIN, HIGH);
	}
}

int16_t i2c_data_buf = UNKNOWN_VAL;

// function that executes whenever data is received from master
void receiveEvent(int howMany)
{
// set register to read
// set temp2set
// start
// stop
// read temp
// read pwm
// read current
// read error
//	Serial.print("Received: ");
//	Serial.print(howMany);
//	Serial.println(" byte(s)");
		byte reg = Wire.read();
//		Serial.print("Register:");
//		Serial.println(reg);
		switch(reg) {
			case I2C_REG_SET_TEMP:
				set_temp2set(Wire.read());
				break;
			case I2C_REG_READ_PWM:
				i2c_data_buf = heater_pwm;
				break;
			case I2C_REG_READ_TEMP:
				i2c_data_buf = temp;
				break;
		}
}

void requestEvent()
{
	Wire.write((byte)((i2c_data_buf>>8)&0xff));
	Wire.write((byte)(i2c_data_buf&0xff));
//	Serial.print(i2c_data_buf);
//	Serial.println(" sent");
	i2c_data_buf = UNKNOWN_VAL;
}

// sample time of PID
Ticker process_timer(process_proc, 100, 0, MILLIS);

Ticker status_timer(display_status, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	pinMode(FAN_PIN, OUTPUT);
	digitalWrite(FAN_PIN, LOW);
	analogReference(INTERNAL);

	Peltier.begin(M1_PWM, M1_INA, M1_INB, M1_DIAG, M1_CS);    // Peltier object connected through specified pins 

	Wire.begin(I2C_SLAVE_ADDRESS);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

	process_timer.start();
	status_timer.start();
}

void loop()
{
	process_timer.update();
	status_timer.update();
}
