#ifndef __SPINNING_H

#define __SPINNING_H

#include "action.h"

class Spinning: public Action {

	protected:
		int heading, stage;

	public:
		Spinning();
		virtual bool loop();

};

#endif //__SPINNING_H
