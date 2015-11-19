#include "adxl345_proc.h"
#include "binary_const.h"
#include <Adafruit_ADXL345_U.h>
#include "limited_queue.h"

#define A_INT1_PIN 5

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel;

ScalarLimitedQueue<float, 10> acc_x;
ScalarLimitedQueue<float, 10> acc_y;
ScalarLimitedQueue<float, 10> acc_z;

adxl345_state_t adxl345_state;

bool adxl345_exists = false;

void setup_accel()
{

	adxl345_state.reset();

	accel = Adafruit_ADXL345_Unified(12345);

	/* Initialise the sensor */
	if(!(adxl345_exists = accel.begin()))
	{
		Serial.println(F("Could not connect to ADXL345."));
	} else {
		Serial.println(F("ADXL345 is ready"));

		// Set the range of the accelerometer to a maximum of 2G.
		accel.setRange(ADXL345_RANGE_2_G);

		// interrupts setup
		pinMode(A_INT1_PIN, INPUT); 

		accel.writeRegister(ADXL345_REG_INT_MAP, 0); // send all interrupts to ADXL345's INT1 pin
		accel.writeRegister(ADXL345_REG_INT_ENABLE, B8(1111100)); // enable signle and double tap, activity, inactivity and free fall detection

		// single tap configuration
		accel.writeRegister(ADXL345_REG_DUR, 0x1f); // 625us/LSB
		accel.writeRegister(ADXL345_REG_THRESH_TAP, 24); // 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
		accel.writeRegister(ADXL345_REG_TAP_AXES, B8(111)); // enable tap detection on x,y,z axes

	}
}

void process_accel()
{
	if(adxl345_exists) {
		accel.getEvent(&adxl345_state.event);
		acc_x.push(adxl345_state.event.acceleration.x);
		acc_y.push(adxl345_state.event.acceleration.y);
		acc_z.push(adxl345_state.event.acceleration.z);

		if(digitalRead(A_INT1_PIN)) {
			int interruptSource = accel.readRegister(ADXL345_REG_INT_SOURCE);
			if(interruptSource & B8(1000000)) {
				adxl345_state.single_tap++;
				Serial.println("### SINGLE_TAP ###");
			}
		}
	}

}

float acc_x_avg()
{
	return acc_x.average(0);
}

float acc_x_max()
{
	return acc_x.vmax(0);
}

float acc_y_avg()
{
	return acc_y.average(0);
}

float acc_y_max()
{
	return acc_y.vmax(0);
}

float acc_z_avg()
{
	return acc_z.average(0);
}

float acc_z_max()
{
	return acc_z.vmax(0);
}
