#include "ADXL345.h"
#include <syslog.h>
#include <math.h>

#define ADXL345_I2C_ADDR 0x53

#define Register_PowerControl 0x2D
#define Register_DataFormat 0x31
#define Register_DataX 0x32
#define Register_DataY 0x34
#define Register_DataZ 0x36

#define ErrorCode_1 "Entered range was invalid. Should be 2, 4, 8 or 16g."
#define ErrorCode_1_Num 1

#define ScaleFor2G 0.0039
#define ScaleFor4G 0.0078
#define ScaleFor8G 0.0156
#define ScaleFor16G 0.0312


ADXL345::ADXL345():ReServant("cmd.adxl345", "adxl345"),m_Scale(1)
{
	const CMD_FUNC cmdlist[] = {};
	this->setCmdList(cmdlist);
}

void ADXL345::create_servant()
{
	ReServant::create_servant();
	/* initialise ADXL345 */
//	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	setRange(2, true);
//	i2cwire.writeToDevice(Register_DataFormat, 0);
//	i2cwire.writeToDevice(Register_DataFormat, 11);
}


int ADXL345::setRange(int gNum, bool fullResolution)
{
	uint8_t data;
	syslog(LOG_NOTICE, "Setting range to: %dg", gNum);

	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	// Get current data from this register.
	i2cwire.requestFromDevice(Register_DataFormat, 1, &data);

	// We AND with 0xF4 to clear the bits are going to set.
	// Clearing ----X-XX
	data &= 0xF4;

	// By default (range 2) or FullResolution = true, scale is 2G.
	int m_Scale = ScaleFor2G;
	
	// Set the range bits.
	switch(gNum)
	{
		case 2:
			break;
		case 4:
			data |= 0x01;
			if(!fullResolution) { m_Scale = ScaleFor4G; }
			break;
		case 8:
			data |= 0x02;
			if(!fullResolution) { m_Scale = ScaleFor8G; }
			break;
		case 16:
			data |= 0x03;
			if(!fullResolution) { m_Scale = ScaleFor16G; }
			break;
		default:
			return ErrorCode_1_Num;
	}

	// Set the full resolution bit.
	if(fullResolution)
		data |= 0x08;

	i2cwire.writeToDevice(Register_DataFormat, data);
	return 0;
}

AccelerometerRaw ADXL345::readRawAxis()
{
	int16_t buf[3];
	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	i2cwire.requestFromDevice(Register_DataX, 6, (uint8_t*)buf);
	AccelerometerRaw raw = AccelerometerRaw();
//	raw.XAxis = (buf[1] << 8) | buf[0];
//	raw.YAxis = (buf[3] << 8) | buf[2];
//	raw.ZAxis = (buf[5] << 8) | buf[4];
	raw.XAxis = buf[0];
	raw.YAxis = buf[1];
	raw.ZAxis = buf[2];
	return raw;
}

AccelerometerScaled ADXL345::readScaledAxis()
{
	AccelerometerRaw raw = readRawAxis();
	AccelerometerScaled scaled = AccelerometerScaled();
	scaled.XAxis = raw.XAxis * m_Scale;
	scaled.YAxis = raw.YAxis * m_Scale;
	scaled.ZAxis = raw.ZAxis * m_Scale;
	return scaled;
}

void ADXL345::enableMeasurements()
{
	syslog(LOG_NOTICE, "Enabling measurements.");
	i2cwire.writeToDevice(Register_PowerControl, 0);
	i2cwire.writeToDevice(Register_PowerControl, 16);
	i2cwire.writeToDevice(Register_PowerControl, 8);
}


float ADXL345::heading(float axis1, float axis2)
{
	float heading = atan2(axis1, axis2);
	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2 * M_PI;
	// Check for wrap due to addition of declination.
	if(heading > 2 * M_PI)
		heading -= 2 * M_PI;
	// Convert radians to degrees for readability.
	return heading * 180 / M_PI;
}

void ADXL345::loop()
{

	AccelerometerRaw data = readRawAxis();
	AccelerometerScaled scaled = readScaledAxis();
	float xz_degrees = heading(scaled.XAxis, scaled.ZAxis);
	float yz_degrees = heading(scaled.YAxis, scaled.ZAxis);

	syslog(LOG_NOTICE, "Raw:%d %4d %4d\n", data.XAxis, data.YAxis, data.ZAxis);
	syslog(LOG_NOTICE, "Scaled:%g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	syslog(LOG_NOTICE, "XZ:%g YZ:%g\n", xz_degrees, yz_degrees);
	redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.x %g", scaled.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.y %g", scaled.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.z %g", scaled.ZAxis);


	redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.xz %g", xz_degrees);
	redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.yz %g", yz_degrees);

	ReServant::loop();
}


