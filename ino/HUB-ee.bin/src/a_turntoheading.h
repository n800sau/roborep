#ifndef __TURNTOHEADING_H

#define __TURNTOHEADING_H

#include "action.h"

class TurnToHeading: public Action {

	protected:
		int heading;

	public:
		TurnToHeading(int heading);

		virtual bool loop();

};


#endif //__TURNTOHEADING_H
