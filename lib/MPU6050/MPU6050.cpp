#include "MPU6050.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

struct MPU6050_CMD_FUNC:public CMD_FUNC {
	public:
		MPU6050_CMD_FUNC(const char *cmd, MPU6050::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		MPU6050::tFunction ptr;
};

typedef MPU6050_CMD_FUNC *pMPU6050_CMD_FUNC;

MPU6050::MPU6050():ReServant("mpu6050")
{
	memset(&raw, 0, sizeof(raw));
	memset(&stop_raw, 0, sizeof(stop_raw));
	const static pMPU6050_CMD_FUNC cmdlist[] = {
		new MPU6050_CMD_FUNC("stop_state", &MPU6050::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void MPU6050::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pMPU6050_CMD_FUNC ccmd = (pMPU6050_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void MPU6050::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
	readRaw(stop_raw);
	int count = 100;
	for (int i = 0; i < count; i++) { // Take the average of 100 readings
		stop_raw += *readRaw(raw);
		usleep(10000);
	}
	stop_raw /= count;
	stop_raw.val.timestamp = dtime();
	const char *reply_key = json_string_value(json_object_get(js, "reply_key"));
	if(reply_key) {
		syslog(LOG_NOTICE, "replied to %s", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s OK", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "EXPIRE %s 30", reply_key);
	} else {
		syslog(LOG_NOTICE, "no reply_key");
	}
}

void MPU6050::create_servant()
{
	uint8_t c;
	ReServant::create_servant();
	/* initialise MPU6050 */
	i2cwire.selectDevice(MPU6050_I2C_ADDRESS, myid());
	int error = i2cwire.requestFromDevice(MPU6050_WHO_AM_I, 1, &c) != 1;
	if(error) {
		printf("Error\n");
	} else {
		printf("WHO_AM_I : %2.2X\n", c);
	}
  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
	if(i2cwire.requestFromDevice(MPU6050_PWR_MGMT_2, 1, &c) != 1) {
		printf("PWR_MGMT_2 : %2.2X", c);
	}
	// Clear the 'sleep' bit to start the sensor
	i2cwire.writeToDevice(MPU6050_PWR_MGMT_1, 0);
}

#define AG_REG16_COUNT AG_VAL_COUNT
#define AG_REG8_COUNT (AG_REG16_COUNT*2)
accel_t_gyro_union *MPU6050::readRaw(accel_t_gyro_union &data)
{
	union {
		uint8_t reg8[AG_REG8_COUNT];
		uint16_t reg16[AG_REG16_COUNT];
	} __attribute__ ((__packed__)) buf;
	i2cwire.selectDevice(MPU6050_I2C_ADDRESS, myid());
	bool ok = i2cwire.requestFromDevice(MPU6050_ACCEL_XOUT_H, AG_REG8_COUNT, buf.reg8);
	if (!ok) {
		syslog(LOG_ERR, "Error reading %s", myid());
	} else {
		for(int i=0; i<AG_REG8_COUNT; i+=2) {
			uint8_t tmp = buf.reg8[i];
			buf.reg8[i] = buf.reg8[i+1];
			buf.reg8[i+1] = tmp;
//			printf("%4.4X ", buf.reg16[i]);
		}
//		printf("\n");
		data.transfer(buf.reg16);
		data.val.timestamp = dtime();
	}
	return &data;
}

float MPU6050::heading(float axis1, float axis2)
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

void MPU6050::fill_json(json_t *js)
{
	json_t *sjs;
	readRaw(raw);

	float xz_degrees = heading(raw.val.x_accel, raw.val.z_accel);
	float yz_degrees = heading(raw.val.y_accel, raw.val.z_accel);

	//syslog(LOG_NOTICE, "Raw:%d %4d %4d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	//syslog(LOG_NOTICE, "Scaled:%g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	//syslog(LOG_NOTICE, "XZ:%g YZ:%g\n", xz_degrees, yz_degrees);
	sjs = json_object();
	json_object_set_new(sjs, "accel_x", json_integer(raw.val.x_accel));
	json_object_set_new(sjs, "accel_y", json_integer(raw.val.y_accel));
	json_object_set_new(sjs, "accel_z", json_integer(raw.val.z_accel));
	json_object_set_new(sjs, "gyro_x", json_integer(raw.val.x_gyro));
	json_object_set_new(sjs, "gyro_y", json_integer(raw.val.y_gyro));
	json_object_set_new(sjs, "gyro_z", json_integer(raw.val.z_gyro));
	json_object_set_new(sjs, "temperature", json_integer(raw.val.temperature));
	json_object_set_new(sjs, "timestamp", json_real(raw.val.timestamp));
	json_object_set_new(js, "raw", sjs);

	sjs = json_object();
	json_object_set_new(sjs, "accel_x", json_integer(stop_raw.val.x_accel));
	json_object_set_new(sjs, "accel_y", json_integer(stop_raw.val.y_accel));
	json_object_set_new(sjs, "accel_z", json_integer(stop_raw.val.z_accel));
	json_object_set_new(sjs, "gyro_x", json_integer(stop_raw.val.x_gyro));
	json_object_set_new(sjs, "gyro_y", json_integer(stop_raw.val.y_gyro));
	json_object_set_new(sjs, "gyro_z", json_integer(stop_raw.val.z_gyro));
	json_object_set_new(sjs, "temperature", json_integer(stop_raw.val.temperature));
	json_object_set_new(sjs, "timestamp", json_real(stop_raw.val.timestamp));
	json_object_set_new(js, "stop_raw", sjs);

	json_object_set_new(js, "xz_degrees", json_real(xz_degrees));
	json_object_set_new(js, "yz_degrees", json_real(yz_degrees));
}

void MPU6050::loop()
{
	json2redislist();
	ReServant::loop();
}
