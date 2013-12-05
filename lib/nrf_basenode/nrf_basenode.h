#ifndef __NRF_BASENODE_H

#define __NRF_BASENODE_H

#include <reservant.h>

class NRF_BASENODE:public ReServant
{
	private:
		int serd;
		struct event *serd_event;

	protected:
		virtual bool create_servant();
		virtual void destroy_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);
		virtual void push_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		NRF_BASENODE(const char datafname[]=NULL);
		virtual ~NRF_BASENODE();
		typedef void (NRF_BASENODE::*tFunction)(json_t *js);
};

#endif //__NRF_BASENODE_H
