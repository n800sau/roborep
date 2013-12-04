#ifndef __MPU6050_H

#define __MPU6050_H

#include <reservant.h>

class OCULUS:public ReServant
{
	protected:
		virtual bool create_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);
		virtual void push_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		OCULUS(const char datafname[]=NULL);
		virtual ~OCULUS();
		typedef void (OCULUS::*tFunction)(json_t *js);
};

#endif //__MPU6050_H
