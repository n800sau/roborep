#include "nrf_basenode.h"
#include "../../ino/include/common.h"
#include "serialCom.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>

#define SERDEV "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0"
#define BAUD B57600

/* Flag indicating datas are ready to be read */
bool NRF_BASENODE::serd_available = false;
 
/* SIGIO handler */
void NRF_BASENODE::serd_handler(int status)
{
  /* Data ready to be read */
  serd_available = true;
}

struct NRF_BASENODE_CMD_FUNC:public CMD_FUNC {
	public:
		NRF_BASENODE_CMD_FUNC(const char *cmd, NRF_BASENODE::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		NRF_BASENODE::tFunction ptr;
};

typedef NRF_BASENODE_CMD_FUNC *pNRF_BASENODE_CMD_FUNC;

NRF_BASENODE::NRF_BASENODE(const char datafname[]):ReServant("nrf_basenode"),serd(0),line_len(0)
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
//		setLoopInterval(5);
		/* Initialize the serial communication */
		serd = serialConfiguration(serd_handler, SERDEV, BAUD, NO_PARITY_CHECK);
	}
	return rs;
}

void NRF_BASENODE::destroy_servant()
{
	ReServant::destroy_servant();
}

bool NRF_BASENODE::fill_json(json_t *js)
{
	bool rs = false;
	if(NRF_BASENODE::serd_available) {
		int count;
		do {
			count = read(serd, &line[line_len], 1);
			if(line[line_len] == '\xa') {
				break;
			}
			line_len += count;
		} while(count > 0 && line_len < sizeof(line)-1);
		NRF_BASENODE::serd_available = false;
		if(line[line_len] == '\xa' || line_len >= sizeof(line)-1) {
			line[line_len] = 0;
			if(strncmp(line, ACCEL_MARKER, sizeof(ACCEL_MARKER)-1) == 0) {
				const char *delimiters = "\t:\xd";
				char *token;
				token = strtok(line + sizeof(ACCEL_MARKER)-1, delimiters);
				json_t *sjs = json_object();
				const char *key = NULL;
				while(token) {
					if(!key) {
						key = token;
					} else {
						json_object_set_new(sjs, key, json_real(atof(token)));
						key = NULL;
					}
					token = strtok(NULL, delimiters);
				}
				json_object_set_new(js, "accel", sjs);
				rs = true;
			}
			line_len = 0;
		}
	}
	return rs;
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
