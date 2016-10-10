#include "hmc5883l_proc.h"

sensors_event_t hmc5883_event;

Adafruit_HMC5883_Unified mag;
float heading = -1;
float headingDegrees = -1;

float compass_x;
float compass_y;
float compass_z;

void setup_compass()
{
	mag = Adafruit_HMC5883_Unified(12345);
	if(!mag.begin()) {
		Serial.println(F("Could not connect to HMC5883."));
	} else {
		Serial.println(F("HMC5883 is ready."));
	}
}

void process_compass()
{
	mag.getEvent(&hmc5883_event);

	// Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	heading = atan2(hmc5883_event.magnetic.y, hmc5883_event.magnetic.x);
	compass_x = hmc5883_event.magnetic.x;
	compass_y = hmc5883_event.magnetic.y;
	compass_z = hmc5883_event.magnetic.z;

	// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/
	// Mine is: 2 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
	// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	float declinationAngle = 0.0457;
	heading += declinationAngle;

	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2*PI;

	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
		heading -= 2*PI;

	// Convert radians to degrees for readability
	headingDegrees = heading * 180/M_PI; 

}
