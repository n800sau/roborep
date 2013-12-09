#ifndef __ROSTF_H

#define __ROSTF_H

#include <reservant.h>
#include "tf/tfMessage.h"

class ROStf:public ReServant
{
	private:
		int argc;
		char **argv;

	protected:
		virtual void loop();
		virtual bool fill_json(json_t *js, int list_id);

	public:
		ROStf(int argc, char **argv);
		virtual bool create_servant();
		void tf_message_received(const tf::tfMessageConstPtr &msg);
};

#endif
