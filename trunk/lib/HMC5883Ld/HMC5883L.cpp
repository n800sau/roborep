#include "HMC5883L.h"
#include <syslog.h>
#include <math.h>


HMC5883L::HMC5883L():ReServant("hmc5883l"),m_Scale(1)
{
}

MagnetometerRaw HMC5883L::ReadRawAxis()
{
  MagnetometerRaw raw = MagnetometerRaw();
  uint8_t buffer[10];
  Read(DataRegisterBegin, buffer, 6);
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

MagnetometerScaled HMC5883L::ReadScaledAxis()
{
  MagnetometerRaw raw = ReadRawAxis();
  MagnetometerScaled scaled = MagnetometerScaled();
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  return scaled;
}

int HMC5883L::SetScale(GAUSS gauss)
{
	uint8_t regValue = 0x00;
	if(gauss == GAUSS_0_88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == GAUSS_1_3)
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == GAUSS_1_9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == GAUSS_2_5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == GAUSS_4_0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == GAUSS_4_7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == GAUSS_5_6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == GAUSS_8_1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	else
		return ErrorCode_1_Num;
	
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	Write(ConfigurationRegisterB, regValue);
	return 0;
}

int HMC5883L::SetMeasurementMode(uint8_t mode)
{
	Write(ModeRegister, mode);
}

void HMC5883L::Write(int address, int data)
{
	i2cwire.selectDevice(HMC5883L_Address, "HMC5883L");
	i2cwire.writeToDevice(address, data);
}

uint8_t* HMC5883L::Read(int address, uint8_t *buf, int length)
{
	i2cwire.selectDevice(HMC5883L_Address, "HMC5883L");
	i2cwire.requestFromDevice(address, length, buf);
	return buf;
}

const char* HMC5883L::GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	
	return "Error not defined.";
}

void HMC5883L::create_servant()
{
	ReServant::create_servant();
}

void HMC5883L::loop()
{
	syslog(LOG_NOTICE, "Setting scale to +/- 1.3 Ga\n");
	int error = SetScale(GAUSS_1_3); // Set the scale of the compass->
	if(error != 0) // If there is an error, print it out.
	  syslog(LOG_ERR, "%d:%s\n", error, GetErrorText(error));
	// Retrive the raw values from the compass (not scaled).
	MagnetometerRaw raw = ReadRawAxis();
	// Retrived the scaled values from the compass (scaled to the configured scale).
	MagnetometerScaled scaled = ReadScaledAxis();

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
		heading += 2 * M_PI;
  
	// Check for wrap due to addition of declination.
	if(heading > 2 * M_PI)
		heading -= 2*M_PI;

	// Convert radians to degrees for readability.
	float headingDegrees = heading * 180/M_PI; 

	syslog(LOG_NOTICE, "Raw: %d %d %d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.raw_x %d", myid(), raw.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.raw_y %d", myid(), raw.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.raw_z %d", myid(), raw.ZAxis);

	syslog(LOG_NOTICE, "Scaled: %g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_x %g", myid(), scaled.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_y %g", myid(), scaled.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_z %g", myid(), scaled.ZAxis);

	syslog(LOG_NOTICE, "Heading: %g radians, %g degrees\n", heading, headingDegrees);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.heading_radians %g", myid(), heading);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.heading_degrees %g", myid(), headingDegrees);

	ReServant::loop();
}

