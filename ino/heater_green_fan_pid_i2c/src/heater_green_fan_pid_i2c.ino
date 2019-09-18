#include <VNH3SP30.h>
#include <Wire.h>
#include <Ticker.h>

const byte I2C_SLAVE_ADDRESS = 8;
const byte I2C_REG_SET_TEMP = 0x01;
const byte I2C_REG_RESTART = 0x02;
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

bool emergency_stop = false;
const int EMERGENCY_LED_PIN = 13;
bool emergency_led_on = false;
unsigned long emergency_last_ts = 0;

int heater_pwm = 0;
const int MIN_PWM = -400, MAX_PWM = 400;

// NTC
const int TEMP_PIN = A2;

// Fan
const int FAN_PIN = 7;

#define UNKNOWN_VAL -1000

double temp = UNKNOWN_VAL, temp2set = UNKNOWN_VAL;
const double max_temp = 100, min_temp = 0;

// for emergency stop test
double old_temp = UNKNOWN_VAL;
int old_heater_pwm = 0;

double Vcc = 3.3;
double Vref = 1.1;
const double T_0 = 273.15;
const double T_25 = T_0 + 25;
// 100k NTC
const double beta = 3950;
const double R_25 = 100000L; // 100k ohm
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

void set_temp2set(int t)
{
	temp2set = t;
#ifdef USE_PID
	if(temp2set < temp) {
		// K for cooling
		heaterPID.SetTunings(cKp, cKi, cKd);
	} else {
		// K for heating
		heaterPID.SetTunings(hKp, hKi, hKd);
	}
#endif // USE_PID
}

void process_proc()
{
	update_temp();
	if(emergency_stop) {
		if(heater_pwm != 0) {
			heater_pwm = 0;
			Serial.println("Emergency stop. Reboot me !!!");
			digitalWrite(FAN_PIN, HIGH);
		}
	} else {
#ifdef USE_PID
		Input = temp;
		Setpoint = temp2set;
//		Serial.print("Error:");
//		Serial.println(temp2set-temp);
		heaterPID.Compute();
		heater_pwm = Output;
//		Serial.print("Output:");
//		Serial.println(heater_pwm);
#else
		if(temp < temp2set) {
			heater_pwm = MAX_PWM;
		} else if(temp > temp2set) {
			heater_pwm = MIN_PWM;
		}
#endif // USE_PID
		if(heater_pwm > MAX_PWM/2) {
			digitalWrite(FAN_PIN, LOW);
		} else if(heater_pwm < 0) {
			digitalWrite(FAN_PIN, HIGH);
		}
	}
	Peltier.setSpeed(heater_pwm);
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
			case I2C_REG_RESTART:
				emergency_stop = false;
				temp = temp2set = UNKNOWN_VAL;
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

void emergency_test()
{
	if(!emergency_stop) {
		if(old_temp == UNKNOWN_VAL) {
			old_temp = temp;
			old_heater_pwm = heater_pwm;
			emergency_last_ts = millis();
		} else {
			if(millis() - emergency_last_ts > 10000) {
				emergency_last_ts = millis();
				if(old_heater_pwm > 10 && heater_pwm > 10 && temp - old_temp <= 0) {
					emergency_stop = true;
				} else if(old_heater_pwm < 10 && heater_pwm < 10 && temp - old_temp >= 0) {
					emergency_stop = true;
				}
				old_temp = temp;
				old_heater_pwm = heater_pwm;
			} else {
				if(old_heater_pwm*heater_pwm <= 0) {
					old_temp = temp;
					old_heater_pwm = heater_pwm;
					emergency_last_ts = millis();
				}
			}
		}
	} else {
		emergency_led_on = !emergency_led_on;
		digitalWrite(EMERGENCY_LED_PIN, emergency_led_on ? HIGH : LOW);
	}
}

// sample time of PID
Ticker process_timer(process_proc, 200, 0, MILLIS);
Ticker status_timer(display_status, 1000, 0, MILLIS);
Ticker emergency_timer(emergency_test, 1000, 0, MILLIS);

void setup()
{
	Serial.begin(115200);
	pinMode(EMERGENCY_LED_PIN, OUTPUT);
	digitalWrite(EMERGENCY_LED_PIN, LOW);
	pinMode(FAN_PIN, OUTPUT);
	digitalWrite(FAN_PIN, LOW);
	analogReference(INTERNAL);

	Peltier.begin(M1_PWM, M1_INA, M1_INB, M1_DIAG, M1_CS);    // Peltier object connected through specified pins 

	Wire.begin(I2C_SLAVE_ADDRESS);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

#ifdef USE_PID
	//tell the PID to range between 0 and the full window size
	heaterPID.SetOutputLimits(MIN_PWM, MAX_PWM);

	//turn the PID on
	heaterPID.SetMode(AUTOMATIC);
#endif // USE_PID

	process_timer.start();
	status_timer.start();
	emergency_timer.start();
}

void loop()
{
	process_timer.update();
	status_timer.update();
	emergency_timer.update();
}
