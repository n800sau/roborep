#ifndef __OCULUS_H

#define __OCULUS_H

#include <reservant.h>

class OCULUS:public ReServant
{
	protected:
		virtual bool create_servant();
		virtual void loop();
		virtual bool fill_json(json_t *js);
		virtual void push_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		OCULUS(const char datafname[]=NULL);
		virtual ~OCULUS();
		typedef void (OCULUS::*tFunction)(json_t *js);
};

#endif //__OCULUS_H
