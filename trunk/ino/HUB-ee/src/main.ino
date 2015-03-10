#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "motor_proc.h"
#include "irdist_proc.h"
#include "presence_proc.h"
#include <voltage.h>

// hmc5883l - 0x1e
// adxl345 - 0x53
// bmp085 - 0x77
// l3g4200d - 0x69


int led = 13;
int lon = 0;

void setup()
{
	pinMode(led, OUTPUT);
	digitalWrite(led, 1);
	Serial.begin(57600);
	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	setup_irdist();
	setup_presence();
	setup_motors();
}

void printState()
{
	int v = readVccMv();
	Serial.print("V:");
	Serial.print(v);
	Serial.print(",head:");
	Serial.print(headingDegrees);
	Serial.print(",acc_x:");
	Serial.print(accel_scaled.XAxis);
	Serial.print(",gyro_x:");
	Serial.print((int)gyro.g.x);
	Serial.println();

	Serial.print("T:");
	Serial.print(bmp.readTemperature());
	Serial.print(" *C");

	Serial.print(", P:");
	Serial.print(bmp.readPressure());
	Serial.print(" Pa");

	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	Serial.print(", Alt:");
	Serial.print(bmp.readAltitude());
	Serial.print(" m");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
	// Sydney 1018.1 hPa
	Serial.print(", R.alt:");
	Serial.print(bmp.readAltitude(101500));
	Serial.print(" m");
	Serial.print(", IR dist:");
	Serial.print(distance);
	Serial.print(", Presence detected:");
	Serial.print(MVcount);

	Serial.println();
}

void loop()
{
	digitalWrite(led, lon);
	lon = !lon;
	process_compass();
	process_accel();
	process_gyro();
	process_bmp085();
	process_irdist();
	process_presence();
	process_motors();

	printState();
	delay(1000);
}
