#include "oculus.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

struct OCULUS_CMD_FUNC:public CMD_FUNC {
	public:
		OCULUS_CMD_FUNC(const char *cmd, OCULUS::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		OCULUS::tFunction ptr;
};

typedef OCULUS_CMD_FUNC *pOCULUS_CMD_FUNC;

OCULUS::OCULUS(const char datafname[]):ReServant("oculus")
{
	const static pOCULUS_CMD_FUNC cmdlist[] = {
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

OCULUS::~OCULUS() {
}

void OCULUS::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pOCULUS_CMD_FUNC ccmd = (pOCULUS_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

bool OCULUS::create_servant()
{
	uint8_t c;
	bool rs = ReServant::create_servant();
	if(rs) {
	}
	return rs;
}

bool OCULUS::fill_json(json_t *js)
{
	return false;
}

#define JSONSTR(key) (json_string_value(json_object_get(js, key)))
#define JSONREAL(key) (json_real_value(json_object_get(js, key)))

void OCULUS::push_json(json_t *js)
{
}

void OCULUS::loop()
{
	json2redislist();
	ReServant::loop();
}
