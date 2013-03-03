#ifndef __BMP085_H

#define __BMP085_H

#include <reservant.h>
#include <I2CWire.h>

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34


class BMP085:public ReServant
{
	private:
		uint8_t oversampling;
		int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
		uint16_t ac4, ac5, ac6;

	protected:
		I2CWire i2cwire;
		virtual void create_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);

	public:
		BMP085();
		void init_mode(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
		float readTemperature();
		float readPressure();
		float readAltitude(float sealevelPressure = 101325); // std atmosphere
		uint16_t readRawTemperature();
		uint32_t readRawPressure();
};

#endif
