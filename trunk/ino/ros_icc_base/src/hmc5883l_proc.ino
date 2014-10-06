#include "hmc5883l_proc.h"
#include <HMC5883L.h>

// Store our compass as a variable.
HMC5883L compass;

MagnetometerRaw compass_raw;
MagnetometerScaled compass_scaled;
int MilliGauss_OnThe_XAxis;
float heading;
// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 2 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
// 12° 30 E : -12.50 degrees or -0.218166 rad
// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//float declinationAngle = 0.0457;
const float declinationAngle = -(2 + 30./60) * PI / 180;
float headingDegrees;

void call_compass(int error)
{
	if(error != 0) {
		Serial.println(compass.GetErrorText(error));
	}
}

void setup_compass()
{
	// Setup compass
	compass = HMC5883L(); // Construct a new HMC5883 compass.
	// Setting scale to +/- 1.3 Ga
	call_compass(compass.SetScale(1.3));
	// Set the measurement mode to Continuous
	call_compass(compass.SetMeasurementMode(Measurement_Continuous));

}

void process_compass()
{
	// Retrive the raw values from the compass (not scaled).
	MagnetometerRaw raw = compass.ReadRawAxis();
	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = compass.ReadScaledAxis();
	
	// Values are accessed like so:
	MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	heading = atan2(scaled.YAxis, scaled.XAxis);
	
	// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/
	// Mine is: 2 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
	// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	heading += declinationAngle;
	
	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2*PI;
		
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
		heading -= 2*PI;
	 
	// Convert radians to degrees for readability.
	headingDegrees = heading * 180/M_PI; 
}
