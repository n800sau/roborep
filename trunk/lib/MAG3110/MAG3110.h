#ifndef __MAG3110_H

#define __MAG3110_H

#include <reservant.h>
#include <I2CWire.h>

#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

class MAG3110:public ReServant
{
	private:
		void stop_state(json_t *js);

	protected:
		I2CWire i2cwire;

		struct vector
		{
			int x, y, z;
			inline vector():x(0),y(0),z(0),timestamp(0) {}
			double timestamp;
			void operator +=(vector val) {
				x += val.x;
				y += val.y;
				z += val.z;
			}
			void operator /=(int val) {
				x /= val;
				y /= val;
				z /= val;
			}
		};

		vector stop_v;

		vector readVector();
		virtual void create_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		MAG3110();
		typedef void (MAG3110::*tFunction)(json_t *js);
};

#endif //__MAG3110_H
