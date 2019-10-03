#ifndef __NRF_BASENODE_H

#define __NRF_BASENODE_H

#include <reservant.h>

struct MELEM;

class NRF_BASENODE:public ReServant
{
	private:
		int serd;
		char line[256];
		int line_len;
		static const MELEM mlist[];
		const MELEM *next_serial_marker();
	protected:
		virtual bool create_servant();
		virtual void destroy_servant();
		virtual void loop();
		virtual const char *list_suffix(int list_id);
		virtual bool fill_json(json_t *js, int list_id);
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
