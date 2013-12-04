#include "nrf_basenode.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

struct NRF_BASENODE_CMD_FUNC:public CMD_FUNC {
	public:
		NRF_BASENODE_CMD_FUNC(const char *cmd, NRF_BASENODE::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		NRF_BASENODE::tFunction ptr;
};

typedef NRF_BASENODE_CMD_FUNC *pNRF_BASENODE_CMD_FUNC;

NRF_BASENODE::NRF_BASENODE(const char datafname[]):ReServant("nrf_basenode")
{
	const static pNRF_BASENODE_CMD_FUNC cmdlist[] = {
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

NRF_BASENODE::~NRF_BASENODE() {
}

void NRF_BASENODE::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pNRF_BASENODE_CMD_FUNC ccmd = (pNRF_BASENODE_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

bool NRF_BASENODE::create_servant()
{
	uint8_t c;
	bool rs = ReServant::create_servant();
	if(rs) {
	}
	return rs;
}

void NRF_BASENODE::fill_json(json_t *js)
{
}

#define JSONSTR(key) (json_string_value(json_object_get(js, key)))
#define JSONREAL(key) (json_real_value(json_object_get(js, key)))

void NRF_BASENODE::push_json(json_t *js)
{
}

void NRF_BASENODE::loop()
{
	json2redislist();
	ReServant::loop();
}
