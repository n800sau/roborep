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

#define LIST_ID_ACCEL 1
#define LIST_ID_MPU 2

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
		set_redis_list_limit(100);
		/* Initialize the serial communication */
		serd = serialConfiguration(serd_handler, SERDEV, BAUD, NO_PARITY_CHECK);
	}
	return rs;
}

void NRF_BASENODE::destroy_servant()
{
	ReServant::destroy_servant();
}

const char *NRF_BASENODE::list_suffix(int list_id)
{
	const char *rs;
	switch(list_id) {
		case LIST_ID_ACCEL:
			rs = ".accel";
			break;
		case LIST_ID_MPU:
			rs = ".mpu";
			break;
		default:
			rs = ReServant::list_suffix(list_id);
			break;
	}
	return rs;
}

bool NRF_BASENODE::fill_json(json_t *js, int list_id)
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
			switch(list_id) {
				case LIST_ID_ACCEL:
					if(strncmp(line, ACCEL_MARKER, sizeof(ACCEL_MARKER)-1) == 0 ) {
						json_object_set_new(js, "accel", tabbed2json(line + sizeof(ACCEL_MARKER)-1));
						rs = true;
					}
					break;
				case LIST_ID_MPU:
					if(strncmp(line, MPU_MARKER, sizeof(MPU_MARKER)-1) == 0 ) {
						json_object_set_new(js, "mpu", tabbed2json(line + sizeof(MPU_MARKER)-1));
						rs = true;
					}
					break;
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
	json2redislist(LIST_ID_ACCEL);
	json2redislist(LIST_ID_MPU);
	ReServant::loop();
}
