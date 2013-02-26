#ifndef __ADXL345_H

#define __ADXL345_H

#include <reservant.h>
#include <I2CWire.h>

struct AccelerometerScaled
{
	AccelerometerScaled() { XAxis = YAxis = ZAxis = 0; timestamp = 0; }
	float XAxis;
	float YAxis;
	float ZAxis;
	double timestamp;
	void operator +=(AccelerometerScaled val) {
		XAxis += val.XAxis;
		YAxis += val.YAxis;
		ZAxis += val.ZAxis;
	}
	void operator /=(int val) {
		XAxis /= val;
		YAxis /= val;
		ZAxis /= val;
	}
};

struct AccelerometerRaw
{
	AccelerometerRaw() { XAxis = YAxis = ZAxis = 0; timestamp = 0; }
	int XAxis;
	int YAxis;
	int ZAxis;
	double timestamp;
	void operator +=(AccelerometerRaw val) {
		XAxis += val.XAxis;
		YAxis += val.YAxis;
		ZAxis += val.ZAxis;
	}
	void operator /=(int val) {
		XAxis /= val;
		YAxis /= val;
		ZAxis /= val;
	}
};


class ADXL345:public ReServant
{
	private:
		float m_Scale;
		AccelerometerRaw raw;
		AccelerometerScaled scaled, stop_scaled;
		float xz_degrees;
		float yz_degrees;

		void stop_state(json_t *js);

	protected:
		I2CWire i2cwire;
		virtual void create_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		ADXL345();
		int setRange(int gNum, bool fullResolution);
		void enableMeasurements();
		float heading(float axis1, float axis2);
		AccelerometerRaw readRawAxis();
		AccelerometerScaled readScaledAxis();
		typedef void (ADXL345::*tFunction)(json_t *js);
};

#endif //__ADXL345_H
