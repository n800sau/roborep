#include "HMC5883L.h"
#include <syslog.h>
#include <math.h>


HMC5883L::HMC5883L():ReServant("hmc5883l"),m_Scale(1)
{
}

MagnetometerRaw HMC5883L::ReadRawAxis()
{
	MagnetometerRaw raw = MagnetometerRaw();
	uint16_t buffer[3];
	i2cwire.selectDevice(HMC5883L_Address, myid());
	i2cwire.requestFromDeviceHL(DataRegisterBegin, 3, buffer);
	raw.XAxis = buffer[0];
	raw.ZAxis = buffer[1];
	raw.YAxis = buffer[2];
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

void HMC5883L::Write(int address, uint8_t data)
{
	i2cwire.selectDevice(HMC5883L_Address, myid());
	i2cwire.writeToDevice(address, data);
}

const char* HMC5883L::GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;
	
	return "Error not defined.";
}

bool HMC5883L::create_servant()
{
	if(ReServant::create_servant()) {
		SetScale();
		SetMeasurementMode(Measurement_Continuous);
	}
}

bool HMC5883L::fill_json(json_t *js)
{
//	syslog(LOG_NOTICE, "Setting scale to +/- 1.3 Ga\n");
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

//	syslog(LOG_NOTICE, "Raw: %d %d %d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	json_object_set_new(js, "rawX", json_integer(raw.XAxis));
	json_object_set_new(js, "rawY", json_integer(raw.YAxis));
	json_object_set_new(js, "rawZ", json_integer(raw.ZAxis));

//	syslog(LOG_NOTICE, "Scaled: %g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	json_object_set_new(js, "scaledX", json_real(scaled.XAxis));
	json_object_set_new(js, "scaledY", json_real(scaled.YAxis));
	json_object_set_new(js, "scaledZ", json_real(scaled.ZAxis));

//	syslog(LOG_NOTICE, "Heading: %g radians, %g degrees\n", heading, headingDegrees);
	json_object_set_new(js, "heading_radians", json_real(heading));
	json_object_set_new(js, "heading_degrees", json_real(headingDegrees));
	return true;
}

void HMC5883L::loop()
{
	json2redislist();
	ReServant::loop();
}
