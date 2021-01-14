/*

am2320
sda - A4
scl - A5

heater pwm - 3

*/

#include <AM2320.h>
#include <PID_v1.h>

AM2320 am2320;

const int HEATER_PIN = 3;

const int MAX_HEAT_VAL = 100;

int heat_val = MAX_HEAT_VAL;

int temp2set = 37;

//Variables PID'll be connected
double Setpoint, Input, Output;

// tuning parameters
double Kp=150, Ki=1, Kd=100;
PID heaterPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup()
{
	Serial.begin(115200);
	am2320.begin();
	pinMode(HEATER_PIN, OUTPUT);
}

void loop()
{
	if (am2320.measure()) {
		float temp = am2320.getTemperature();
		Input = temp;
		Setpoint = temp2set;
		heaterPID.Compute();
		heat_val = Output < 0 ? 0 : (Output > MAX_HEAT_VAL ? MAX_HEAT_VAL : Output);
		analogWrite(HEATER_PIN, heat_val);
		Serial.print("\n\nHeat val: ");
		Serial.println(heat_val);
		Serial.print("Temperature: ");
		Serial.println(temp);
		Serial.print("Humidity: ");
		Serial.println(am2320.getHumidity());
	}
	else
	{
		analogWrite(HEATER_PIN, 0);
		int errorCode = am2320.getErrorCode();
		switch (errorCode)
		{
			case 1: Serial.println("ERR: am2320 is offline"); break;
			case 2: Serial.println("ERR: CRC validation failed."); break;
		}
	}
	delay(500);
}
