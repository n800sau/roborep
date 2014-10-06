#include "adxl345_proc.h"

ADXL345 accel;

AccelerometerRaw accel_raw;
AccelerometerScaled accel_scaled;

void setup_accel()
{
	accel = ADXL345();

	// Check that the accelerometer is infact connected.
	if(!accel.EnsureConnected())
	{
		Serial.println("Could not connect to ADXL345.");
	}

	// Set the range of the accelerometer to a maximum of 2G.
	accel.SetRange(2, true);
	// Tell the accelerometer to start taking measurements.
	accel.EnableMeasurements();
	memset(&accel_raw, 0, sizeof(accel_raw));
	memset(&accel_scaled, 0, sizeof(accel_scaled));
}

void process_accel()
{
	if(accel.IsConnected) {
		accel_raw = accel.ReadRawAxis();
		// Read the *scaled* data from the accelerometer (this does it's own read from the accelerometer
		// so you don't have to ReadRawAxis before you use this method).
		// This useful method gives you the value in G thanks to the Love Electronics library.
		accel_scaled = accel.ReadScaledAxis();
	}
}
