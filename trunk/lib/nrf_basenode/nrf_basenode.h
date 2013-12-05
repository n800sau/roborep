#ifndef __NRF_BASENODE_H

#define __NRF_BASENODE_H

#include <reservant.h>

class NRF_BASENODE:public ReServant
{
	private:
		int serd;
		char line[256];
		int line_len;
	protected:
		virtual bool create_servant();
		virtual void destroy_servant();
		virtual void loop();
		virtual bool fill_json(json_t *js);
		virtual void push_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		NRF_BASENODE(const char datafname[]=NULL);
		virtual ~NRF_BASENODE();
		typedef void (NRF_BASENODE::*tFunction)(json_t *js);
		static void serd_handler(int status);
		static bool serd_available;
};

#endif //__NRF_BASENODE_H
