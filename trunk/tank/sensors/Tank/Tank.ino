#include <Wire.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <L3G4200D.h>
#include <DS1307.h>

#define I2C_Address 2

DS1307 rtc;
L3G4200D gyro;
BMP085 bmp;
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
ADXL345 accel;
// Set up a pin we are going to use to indicate our status using an LED.
int statusPin = 2; // I'm using digital pin 2.

union Float {
	float val;
	byte b[sizeof(float)];
};

enum REGISTERS {R_tC=0, R_pressure};

byte register = R_tC;

float tC; //temperature
float pressure; //pressure

void setup() {
  Serial.begin(9600);
  // Ready an LED to indicate our status.
  pinMode(statusPin, OUTPUT);

  Wire.begin(I2C_Address);
	Wire.onRequest(handleRequest);
	Wire.onReceive(handleReceive);

  bmp.begin();
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  // Create an instance of the accelerometer on the default address (0x1D)
  accel = ADXL345();
  // Check that the accelerometer is infact connected.
  if(accel.EnsureConnected())
  {
    Serial.println("Connected to ADXL345.");
    digitalWrite(statusPin, HIGH); // If we are connected, light our status LED.
  }
  else 
  {
    Serial.println("Could not connect to ADXL345.");
    digitalWrite(statusPin, LOW); // If we are not connected, turn our LED off.
  }
  
  // Set the range of the accelerometer to a maximum of 2G.
  accel.SetRange(2, true);
  // Tell the accelerometer to start taking measurements.
  accel.EnableMeasurements();
  gyro = L3G4200D();
  gyro.enableDefault();
  rtc = DS1307();
  rtc.stop();
  rtc.set(DS1307_SEC,1);        //set the seconds
  rtc.set(DS1307_MIN,23);     //set the minutes
  rtc.set(DS1307_HR,12);       //set the hours
  rtc.set(DS1307_DOW,4);       //set the day of the week
  rtc.set(DS1307_DATE,5);       //set the date
  rtc.set(DS1307_MTH,3);        //set the month
  rtc.set(DS1307_YR,9);         //set the year
  rtc.start();

}

void loop() {
	showBmp();
	showCompass();
	showAccel();
	showGyro();
	showTime();
    Serial.println();
    delay(500);
}

void showBmp()
{
	tC = bmp.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(tC);
    Serial.println(" *C");
    
	pressure = bmp.readPressure()
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");

}

void showCompass()
{
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
  float declinationAngle = 0.0457;
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
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("   ");   
   Serial.print(raw.ZAxis);
   Serial.print("   \tScaled:\t");
   
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);

   Serial.print("   \tHeading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   Serial.print(headingDegrees);
   Serial.println(" Degrees   \t");
}

void showAccel()
{
  if(accel.IsConnected) // If we are connected to the accelerometer.
  {
    // Read the raw data from the accelerometer.
    AccelerometerRaw raw = accel.ReadRawAxis();
    //This data can be accessed like so:
    int xAxisRawData = raw.XAxis;
    
    // Read the *scaled* data from the accelerometer (this does it's own read from the accelerometer
    // so you don't have to ReadRawAxis before you use this method).
    // This useful method gives you the value in G thanks to the Love Electronics library.
    AccelerometerScaled scaled = accel.ReadScaledAxis();
    // This data can be accessed like so:
    float xAxisGs = scaled.XAxis;
    
    // We output our received data.
   // Tell us about the raw values coming from the accelerometer.
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("   ");   
   Serial.print(raw.ZAxis);
   
   // Tell us about the this data, but scale it into useful units (G).
   Serial.print("   \tScaled:\t");
   Serial.print(scaled.XAxis);
   Serial.print("G   ");   
   Serial.print(scaled.YAxis);
   Serial.print("G   ");   
   Serial.print(scaled.ZAxis);
   Serial.println("G");
  }
}

void showGyro()
{
	if(gyro.read())
	{
		Serial.print("G ");
		Serial.print("X: ");
		Serial.print((int)gyro.g.x);
		Serial.print(" Y: ");
		Serial.print((int)gyro.g.y);
		Serial.print(" Z: ");
		Serial.println((int)gyro.g.z);
	} else {
		Serial.println("Error reading gyro");
	}
}

void showTime()
{
  Serial.print(rtc.get(DS1307_HR,true)); //read the hour and also update all the values by pushing in true
  Serial.print(":");
  Serial.print(rtc.get(DS1307_MIN,false));//read minutes without update (false)
  Serial.print(":");
  Serial.print(rtc.get(DS1307_SEC,false));//read seconds
  Serial.print("      ");                 // some space for a more happy life
  Serial.print(rtc.get(DS1307_DATE,false));//read date
  Serial.print("/");
  Serial.print(rtc.get(DS1307_MTH,false));//read month
  Serial.print("/");
  Serial.print(rtc.get(DS1307_YR,false)); //read year 
  Serial.println();
}

void handleRequest()
{
	Float buf[1];
	switch(register) {
		case R_tC:
			buf[0] = tC;
			break;
		case R_pressure:
			buf[0] = pressure;
			break;
		default:
			buf[0] = -1;
			break;
	}
	Wire.send(buf, sizeof(buf));
}

void handleReceive(int howMany)
{
  if(Wire.available())
  {
    register = Wire.receive();
  }
}
