#include <voltage.h>
#include <Wire.h>
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "motor_proc.h"
#include "commands.h"
#include <SerialProtocol.h>

// time in millisecs to stop if no encoder reading
#define TIME2STOP 10000

float stop_acc_x = 0;
float stop_acc_y = 0;
float stop_acc_z = 0;

class IccBase: public SerialProtocol {
	public:
		void sendState();
		void ok();

};

void IccBase::sendState()
{
	float vals[3];
	vals[0] = readVccMv() / 1000.;
	sendFloats(R_VOLTS_1F, vals, 1);
	vals[0] = lCounter;
	vals[1] = rCounter;
	sendFloats(R_MCOUNTS_2F, vals, 2);
	vals[0] = headingDegrees;
	sendFloats(R_HEADING_1F, vals, 1);
	vals[0] = acc_x_avg() - stop_acc_x;
	vals[1] = acc_y_avg() - stop_acc_y;
	vals[2] = acc_z_avg() - stop_acc_z;
	sendFloats(R_ACCAVG_3F, vals, 3);
	vals[0] = acc_x_max() - stop_acc_x;
	vals[1] = acc_y_max() - stop_acc_y;
	vals[2] = acc_z_max() - stop_acc_z;
	sendFloats(R_ACCMAX_3F, vals, 3);
	vals[0] = adxl345_state.single_tap;
	sendFloats(R_HIT_1F, vals, 1);
	vals[0] = gyro.g.x;
	vals[1] = gyro.g.y;
	vals[2] = gyro.g.z;
	sendFloats(R_GYRO_3F, vals, 3);
	bmp.getTemperature(vals);
	sendFloats(R_TEMPERATURE_1F, vals, 1);
	vals[0] = bmp085_event.pressure;
	sendFloats(R_PRESSURE_1F, vals, 1);
	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	// you can get a more precise measurement of altitude
	// if you know the current sea level pressure which will
	// vary with weather and such. If it is 1015 millibars
	// that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	vals[0] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, bmp085_event.pressure);
	sendFloats(R_ALT_1F, vals, 1);
	vals[0] = lPower;
	vals[1] = rPower;
	sendFloats(R_POWER_2F, vals, 2);
	sendFloats(R_END, vals, 0);

	adxl345_state.reset();

}

void IccBase::ok()
{
	sendSimple(R_OK_0);
}


IccBase base;

void setup()
{
	Serial.begin(115200);

	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_bmp085();
	setup_accel();
	setup_gyro();

	motor_setup();
	Serial.println("Setup finished.");
}

void set_power()
{
	// first byte of data is power
	if(*base.datasize >= 1) {
		max_motor_power = (unsigned)base.dataBuf[0];
		min_motor_power = (unsigned)(max_motor_power * 0.8);
		Serial.print("power=");
		Serial.println(max_motor_power);
	}
}

int get_steps()
{
	int rs = 1;
	// first byte of data is power
	if(*base.datasize >= 2) {
		rs = (unsigned)base.dataBuf[1];
		Serial.print("steps=");
		Serial.println(rs);
	}
	return rs;
}

void execute()
{
	switch(*base.command) {
		case C_STATE:
			base.sendState();
			break;
		case C_STOP:
			stop();
			base.ok();
			break;
		case C_FORWARD:
			set_power();
			mv_forward(get_steps());
			base.ok();
			break;
		case C_BACK:
			set_power();
			mv_back();
			base.ok();
			break;
		case C_TLEFT:
			set_power();
			turn_left();
			base.ok();
			break;
		case C_TRIGHT:
			set_power();
			turn_right();
			base.ok();
			break;
	}
}

int led = 13;
int lon = 0;
unsigned long last_time = millis();

void serialEvent() {
	base.serialEvent();
}

void loop()
{
	unsigned long t = millis();
	if(abs(t - last_time) > 1000) {
		last_time = t;
		digitalWrite(led, lon);
		lon = !lon;
//		Serial.println(F("tick"));
	}

	process_compass();
	process_accel();
	process_gyro();
	process_bmp085();
	motor_process();

//		Serial.println('tick');
	if(base.available()) {
//		Serial.println('available');
		execute();
		base.resetInput();
	}

}

