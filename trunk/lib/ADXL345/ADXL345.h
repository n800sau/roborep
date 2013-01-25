#ifndef __ADXL345_H

#define __ADXL345_H

#include <reservant.h>
#include <I2CWire.h>

struct AccelerometerScaled
{
	AccelerometerScaled() { XAxis = YAxis = ZAxis = 0; }
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct AccelerometerRaw
{
	AccelerometerRaw() { XAxis = YAxis = ZAxis = 0; }
	int XAxis;
	int YAxis;
	int ZAxis;
};


class ADXL345:public ReServant
{
	private:
		float m_Scale;
		AccelerometerRaw rawdata;
		AccelerometerScaled scaled;
		float xz_degrees;
		float yz_degrees;
	protected:
		I2CWire i2cwire;
		virtual void create_servant();
		virtual void loop();
	public:
		ADXL345();
		int setRange(int gNum, bool fullResolution);
		void enableMeasurements();
		float heading(float axis1, float axis2);
		AccelerometerRaw readRawAxis();
		AccelerometerScaled readScaledAxis();

		virtual void http_request(struct evhttp_request *req);
};

#endif //__ADXL345_H
