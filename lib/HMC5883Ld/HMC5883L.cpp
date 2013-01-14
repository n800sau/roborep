#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include "HMC5883L.h"


HMC5883L::HMC5883L()
{
  m_Scale = 1;
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

