#include "BMP085.h"
#include <unistd.h>
#include <math.h>
#include <syslog.h>

BMP085::BMP085():ReServant("bmp085") {
}

bool BMP085::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		init_mode();
	}
	return rs;
}


void BMP085::init_mode(uint8_t mode)
{
	oversampling = (mode > BMP085_ULTRAHIGHRES) ? BMP085_ULTRAHIGHRES : mode;

	// read calibration data
	i2cwire.selectDevice(BMP085_I2CADDR, myid());
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC1, 1, (uint16_t*)&ac1);
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC2, 1, (uint16_t*)&ac2);
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC3, 1, (uint16_t*)&ac3);
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC4, 1, (uint16_t*)&ac4);
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC5, 1, (uint16_t*)&ac5);
	i2cwire.requestFromDeviceHL(BMP085_CAL_AC6, 1, (uint16_t*)&ac6);
	i2cwire.requestFromDeviceHL(BMP085_CAL_B1, 1, (uint16_t*)&b1);
	i2cwire.requestFromDeviceHL(BMP085_CAL_B2, 1, (uint16_t*)&b2);
	i2cwire.requestFromDeviceHL(BMP085_CAL_MB, 1, (uint16_t*)&mb);
	i2cwire.requestFromDeviceHL(BMP085_CAL_MC, 1, (uint16_t*)&mc);
	i2cwire.requestFromDeviceHL(BMP085_CAL_MD, 1, (uint16_t*)&md);
}

uint16_t BMP085::readRawTemperature(void)
{
	i2cwire.selectDevice(BMP085_I2CADDR, myid());
	i2cwire.writeToDevice(BMP085_CONTROL, BMP085_READTEMPCMD);
	usleep(5000);
	uint16_t rs;
	i2cwire.requestFromDeviceHL(BMP085_TEMPDATA, 1, &rs);
	return rs;
}

uint32_t BMP085::readRawPressure(void)
{
	i2cwire.selectDevice(BMP085_I2CADDR, "BMP085");
	i2cwire.writeToDevice(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

	// Wait for conversion, delay time dependent on oversampling setting
	if (oversampling == BMP085_ULTRALOWPOWER) {
		usleep(5000);
	} else if (oversampling == BMP085_STANDARD) {
		usleep(8000);
	} else if (oversampling == BMP085_HIGHRES) {
		usleep(14000);
	} else {
		usleep(26000);
	}

	union {
		uint32_t v;
		uint8_t bytes[sizeof(uint32_t)];
	} rs;
	uint8_t buf[3];
	i2cwire.requestFromDevice(BMP085_PRESSUREDATA, 3, buf);
	rs.v = 0;
	rs.bytes[2] = buf[0];
	rs.bytes[1] = buf[1];
	rs.bytes[0] = buf[2];
	rs.v = rs.v >> (8 - oversampling);
//	syslog(LOG_NOTICE, "Oversampling: %d, Raw pressure: %8.8X", oversampling, rs.v);
	return rs.v;
}


float BMP085::readPressure(void)
{
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;

	UT = readRawTemperature();
	UP = readRawPressure();

#if BMP085_DEBUG == 1
	// use datasheet numbers!
	UT = 27898;
	UP = 23843;
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
	b1 = 6190;
	b2 = 4;
	ac3 = -14383;
	ac2 = -72;
	ac1 = 408;
	ac4 = 32741;
	oversampling = 0;
#endif

	// do temperature calculations
	X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
	X2 = ((int32_t)mc << 11) - (X1 + md)/2;     // round up
	X2 /= (X1 + md);
	B5 = X1 + X2;

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

	X1 = ((int32_t)ac3 * B6) >> 13;
	X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + (int32_t)3791)>>4);
	return p/100.;
}


float BMP085::readTemperature(void)
{
	int32_t UT, X1, X2, B5;     // following ds convention
	float temp;

	UT = readRawTemperature();

#if BMP085_DEBUG == 1
	// use datasheet numbers!
	UT = 27898;
	ac6 = 23153;
	ac5 = 32757;
	mc = -8711;
	md = 2868;
#endif

	// step 1
	X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
	X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
	B5 = X1 + X2;
	temp = (B5 + 8) >> 4;
	temp /= 10;

	return temp;
}

float BMP085::readAltitude(float sealevelPressure)
{
	float pressure = readPressure() * 100;
	float altitude = 44330 * (1.0 - pow(pressure /sealevelPressure, 0.1903));

	return altitude;
}

bool BMP085::fill_json(json_t *js)
{
	float t = readTemperature();
	float p = readPressure();
	float a = readAltitude();

//	syslog(LOG_NOTICE, "t=%g, p=%g, a=%g", t, p, a);

	json_object_set_new(js, "temperature", json_real(t));

	json_object_set_new(js, "pressure", json_real(p));

	json_object_set_new(js, "altitude", json_real(a));

	return true;
}

void BMP085::loop()
{
	json2redislist();
	ReServant::loop();
}

