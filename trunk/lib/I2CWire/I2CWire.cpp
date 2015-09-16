#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include "I2CWire.h"

I2CWire::I2CWire()
{
   if ((fd = open(I2CDEV, O_RDWR)) < 0)
   {
      // Open port for reading and writing
      fprintf(stderr, "Failed to open i2c bus\n");
      exit(1);
   }
}

void I2CWire::selectDevice(uint8_t addr, const char * name)
{
   if (ioctl(fd, I2C_SLAVE, addr) < 0)
   {
      fprintf(stderr, "%s not present\n", name);
      //exit(1);
   }
}

int I2CWire::requestFromDevice(uint8_t addr, int length, uint8_t buf[])
{
	if (write(fd, &addr, 1) != 1)
	{
		// Send the register to read from
		fprintf(stderr, "Error writing to i2c slave\n");
		//exit(1);
	}

	int rs = read(fd, buf, length);
	if( rs != length) {
		fprintf(stderr, "Error reading from i2c slave (received %d != requested %d)\n", rs, length);
		rs = 0;
	}
	return rs;
}

int I2CWire::requestFromDeviceHL(uint8_t addr, int length, uint16_t buf[])
{
	int rs = requestFromDevice(addr, length * 2, (uint8_t *)buf);
	if(rs > 0) {
		for(int i=0; i<length; i++) {
			buf[i] = ((buf[i]<<8) & 0xFF00) | ((buf[i]>>8) & 0xFF);
		}
	}
	return rs;
}

int I2CWire::requestFromDeviceLH(uint8_t addr, int length, uint16_t buf[])
{
	return requestFromDevice(addr, length * 2, (uint8_t *)buf);
}

bool I2CWire::writeToDevice(uint8_t reg, uint8_t val)
{
	uint8_t buf[2];
	buf[0]=reg; buf[1]=val;
	bool rs = write(fd, buf, 2) == 2;
	if (!rs)
	{
		fprintf(stderr, "Can't write to i2c slave\n");
	}
	return rs;
}

