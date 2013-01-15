#include "ADXL345.h"
#include <syslog.h>

#define ADXL345_I2C_ADDR 0x53


ADXL345::ADXL345():ReServant("cmd.adxl345", "adxl345")
{
	const CMD_FUNC cmdlist[] = {};
	this->setCmdList(cmdlist);
}

void ADXL345::create_servant()
{
	ReServant::create_servant();
	/* initialise ADXL345 */
	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	i2cwire.writeToDevice(0x2d, 0);
	i2cwire.writeToDevice(0x2d, 16);
	i2cwire.writeToDevice(0x2d, 8);
	i2cwire.writeToDevice(0x31, 0);
	i2cwire.writeToDevice(0x31, 11);
}

void ADXL345::loop()
{
	short x, y, z;
	float xa, ya, za;
	unsigned char buf[16];
	ReServant::loop();
	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");

	if (i2cwire.requestFromDevice(0x32, 6, buf) != 6)
	{
		 //	 X, Y, Z accelerations

		 fprintf(stderr, "Unable to read from ADXL345\n");
		 //exit(1);
	}
	else
	{
		 x = ((unsigned)buf[1])<<8| buf[0];
		 y = ((unsigned)buf[3])<<8| buf[2];
		 z = ((unsigned)buf[5])<<8| buf[4];
		 xa = (90.0 / 256.0) * (float) x;
		 ya = (90.0 / 256.0) * (float) y;
		 za = (90.0 / 256.0) * (float) z;

		syslog(LOG_NOTICE, "%4.0f %4.0f %4.0f\n", xa, ya, za);
		redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.x %g", xa);
		redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.y %g", ya);
		redisAsyncCommand(aredis, NULL, NULL, "SET adxl345.z %g", za);
	}
}


