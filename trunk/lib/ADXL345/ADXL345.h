#ifndef __ADXL345_H

#define __ADXL345_H

#include <reservant.h>
#include <I2CWire.h>


class ADXL345:public ReServant
{
	protected:
		I2CWire i2cwire;
		virtual void create_servant();
		virtual void loop();
	public:
		ADXL345();
};

#endif //__ADXL345_H
