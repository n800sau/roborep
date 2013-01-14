#include <stdio.h>
#include <math.h>
#include "HMC5883L.h"


const double PI  =3.141592653589793238462;

// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   printf("Raw:\t%d\t%d\t%d\n", raw.XAxis, raw.YAxis, raw.ZAxis);

   printf("Scaled:\t%g\t%g\t%g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);

   printf("Heading:\t%g radians,\t%g degrees\n", heading, headingDegrees);
}

int main()
{
	HMC5883L compass = HMC5883L(); // Construct a new HMC5883 compass.


  printf("Setting scale to +/- 1.3 Ga\n");
  int error = compass.SetScale(GAUSS_1_3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    printf("%d:%s\n", error, compass.GetErrorText(error));
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	//12 + 34/60E = 12.56667 / 180 * pi() = 0.2193297
  float declinationAngle = 0.2193297;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);
	return 0;
}
