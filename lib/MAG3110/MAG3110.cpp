#include "MAG3110.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>

struct MAG3110_CMD_FUNC:public CMD_FUNC {
	public:
		MAG3110_CMD_FUNC(const char *cmd, MAG3110::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		MAG3110::tFunction ptr;
};

typedef MAG3110_CMD_FUNC *pMAG3110_CMD_FUNC;

MAG3110::MAG3110():ReServant("mag3110")
{
	const static pMAG3110_CMD_FUNC cmdlist[] = {
		new MAG3110_CMD_FUNC("stop_state", &MAG3110::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void MAG3110::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pMAG3110_CMD_FUNC ccmd = (pMAG3110_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void MAG3110::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
	MAG3110::vector v = readVector();
	int count = 100;
	for (int i = 0; i < count; i++) { // Take the average of 100 readings
		stop_v += readVector();
		usleep(10000);
	}
	stop_v /= count;
	stop_v.timestamp = dtime();
	const char *reply_key = json_string_value(json_object_get(js, "reply_key"));
	if(reply_key) {
		syslog(LOG_NOTICE, "replied to %s", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s OK", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "EXPIRE %s 30", reply_key);
	} else {
		syslog(LOG_NOTICE, "no reply_key");
	}
}

bool MAG3110::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		/* initialise MAG3110 */
		i2cwire.selectDevice(MAG_ADDR, myid());
		// cntrl register2, send 0x80, enable auto resets
		i2cwire.writeToDevice(0x11, 0x80);
		usleep(15000);
		// cntrl register1, send 0x01, active mode
		i2cwire.writeToDevice(0x10, 1);
	}
	return rs;
}


bool MAG3110::fill_json(json_t *js, int list_id)
{
	json_t *sjs;
	MAG3110::vector v = readVector();

	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	float heading = atan2(v.y, v.x);

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

	//adjustment
	heading -= M_PI / 2;

	// Convert radians to degrees for readability.
	float headingDegrees = heading * 180/M_PI; 

	//syslog(LOG_NOTICE, "Raw:%d %4d %4d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	//syslog(LOG_NOTICE, "Scaled:%g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	//syslog(LOG_NOTICE, "XZ:%g YZ:%g\n", xz_degrees, yz_degrees);
	sjs = json_object();
	json_object_set_new(sjs, "x", json_integer(v.x));
	json_object_set_new(sjs, "y", json_integer(v.y));
	json_object_set_new(sjs, "z", json_integer(v.z));
	json_object_set_new(sjs, "timestamp", json_real(v.timestamp));
	json_object_set_new(js, "raw", sjs);

	sjs = json_object();
	json_object_set_new(sjs, "x", json_integer(stop_v.x));
	json_object_set_new(sjs, "y", json_integer(stop_v.y));
	json_object_set_new(sjs, "z", json_integer(stop_v.z));
	json_object_set_new(sjs, "timestamp", json_real(stop_v.timestamp));
	json_object_set_new(js, "stop_raw", sjs);

	json_object_set_new(js, "heading_radians", json_real(heading));
	json_object_set_new(js, "heading_degrees", json_real(headingDegrees));
	return true;
}

void MAG3110::loop()
{
	json2redislist();
	ReServant::loop();
}

#define AG_REG16_COUNT 3
#define AG_REG8_COUNT (AG_REG16_COUNT*2)
MAG3110::vector MAG3110::readVector()
{
	MAG3110::vector rs;
	union {
		uint8_t reg8[AG_REG8_COUNT];
		uint16_t reg16[AG_REG16_COUNT];
	} __attribute__ ((__packed__)) buf;
	i2cwire.selectDevice(MAG_ADDR, myid());
	i2cwire.requestFromDevice(0x1, AG_REG8_COUNT, buf.reg8);
	rs.timestamp = dtime();
	for(int i=0; i<AG_REG8_COUNT; i+=2) {
		uint8_t tmp = buf.reg8[i];
		buf.reg8[i] = buf.reg8[i+1];
		buf.reg8[i+1] = tmp;
	}
	rs.x = buf.reg16[0];
	rs.y = buf.reg16[1];
	rs.z = buf.reg16[2];
	return rs;
}

