#include "ADXL345.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

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

struct ADXL345_CMD_FUNC:public CMD_FUNC {
	public:
		ADXL345_CMD_FUNC(const char *cmd, ADXL345::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		ADXL345::tFunction ptr;
};

typedef ADXL345_CMD_FUNC *pADXL345_CMD_FUNC;

ADXL345::ADXL345():ReServant("adxl345"),m_Scale(1),raw(),scaled(),stop_scaled(),xz_degrees(0),yz_degrees(0)
{
	const static pADXL345_CMD_FUNC cmdlist[] = {
		new ADXL345_CMD_FUNC("stop_state", &ADXL345::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void ADXL345::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pADXL345_CMD_FUNC ccmd = (pADXL345_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void ADXL345::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
	stop_scaled = AccelerometerScaled();
	int count = 100;
	for (int i = 0; i < count; i++) { // Take the average of 100 readings
		stop_scaled += readScaledAxis();
		usleep(10000);
	}
	stop_scaled /= count;
	stop_scaled.timestamp = dtime();
	const char *reply_key = json_string_value(json_object_get(js, "reply_key"));
	if(reply_key) {
		syslog(LOG_NOTICE, "replied to %s", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s OK", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "EXPIRE %s 30", reply_key);
	} else {
		syslog(LOG_NOTICE, "no reply_key");
	}
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
	raw.timestamp = dtime();
	raw.XAxis = buf[0];
	raw.YAxis = buf[1];
	raw.ZAxis = buf[2];
	return raw;
}

AccelerometerScaled ADXL345::readScaledAxis()
{
	AccelerometerRaw raw = readRawAxis();
	AccelerometerScaled scaled = AccelerometerScaled();
	scaled.timestamp = dtime();
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

void ADXL345::fill_json(json_t *js)
{
	json_t *sjs;
	raw = readRawAxis();
	scaled = readScaledAxis();
	xz_degrees = heading(scaled.XAxis, scaled.ZAxis);
	yz_degrees = heading(scaled.YAxis, scaled.ZAxis);

	//syslog(LOG_NOTICE, "Raw:%d %4d %4d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	//syslog(LOG_NOTICE, "Scaled:%g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	//syslog(LOG_NOTICE, "XZ:%g YZ:%g\n", xz_degrees, yz_degrees);
	sjs = json_object();
	json_object_set_new(sjs, "x", json_integer(raw.XAxis));
	json_object_set_new(sjs, "y", json_integer(raw.YAxis));
	json_object_set_new(sjs, "z", json_integer(raw.ZAxis));
	json_object_set_new(sjs, "timestamp", json_real(raw.timestamp));
	json_object_set_new(js, "raw", sjs);

	sjs = json_object();
	json_object_set_new(sjs, "x", json_real(scaled.XAxis));
	json_object_set_new(sjs, "y", json_real(scaled.YAxis));
	json_object_set_new(sjs, "z", json_real(scaled.ZAxis));
	json_object_set_new(sjs, "timestamp", json_real(scaled.timestamp));
	json_object_set_new(js, "scaled", sjs);

	sjs = json_object();
	json_object_set_new(sjs, "x", json_real(stop_scaled.XAxis));
	json_object_set_new(sjs, "y", json_real(stop_scaled.YAxis));
	json_object_set_new(sjs, "z", json_real(stop_scaled.ZAxis));
	json_object_set_new(sjs, "timestamp", json_real(stop_scaled.timestamp));
	json_object_set_new(js, "stop_scaled", sjs);

	json_object_set_new(js, "xz_degrees", json_real(xz_degrees));
	json_object_set_new(js, "yz_degrees", json_real(yz_degrees));
}

void ADXL345::loop()
{
	json2redislist();
	ReServant::loop();
}

int fsize(FILE *fp){
    int prev=ftell(fp);
    fseek(fp, 0L, SEEK_END);
    int sz=ftell(fp);
    fseek(fp,prev,SEEK_SET); //go back to where we were
    return sz;
}
