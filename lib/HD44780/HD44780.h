#ifndef __HD44780_H

#define __HD44780_H

#include <reservant.h>
#ifdef BB
#include "BeagleBone_gpio.h"
#include "BeagleBone_hd44780.h"
#endif //BB

class HD44780:public ReServant
{
	private:

		void stop_state(json_t *js);
		struct gpioID enabled_gpio[6];
		const char *listpath;

	protected:
		virtual void loop();
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		HD44780();
		~HD44780();
		virtual bool create_servant();
		typedef void (HD44780::*tFunction)(json_t *js);
		void setListPath(const char *listpath);
};

#endif //__HD44780_H
