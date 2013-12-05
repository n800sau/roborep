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
#include <termios.h>

#define SERDEV "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0"
#define BAUD B57600

static void nrf_basenode_serd_cb_func(int fd, short int fields, void *ctx)
{
	NRF_BASENODE *ths = (NRF_BASENODE*)ctx;
	char buf[256];
	int i;
	for(i=0; i < sizeof(buf) - 1; i++) {
		char c = read(fd, buf+i, 1);
		syslog(LOG_ERR, "Hello %c", c);
		if(c == '\n') {
			break;
		}
	}
	buf[i] = 0;
	syslog(LOG_INFO, "Hello %s", buf);
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

NRF_BASENODE::NRF_BASENODE(const char datafname[]):ReServant("nrf_basenode"),serd(0),serd_event(0)
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
		serd = open(SERDEV, O_RDWR| O_NOCTTY | O_NDELAY);
		struct termios tty;
		struct termios tty_old;
		memset (&tty, 0, sizeof(tty));

		/* Error Handling */
		if(tcgetattr(serd, &tty) != 0) {
			syslog(LOG_ERR, "Error %d from tcgetattr: %s", errno, strerror(errno));
		}
		/* Save old tty parameters */
		tty_old = tty;

		/* Set Baud Rate */
		cfsetospeed(&tty, (speed_t)BAUD);
		cfsetispeed(&tty, (speed_t)BAUD);

		/* Setting other Port Stuff */
		tty.c_cflag     &=  ~PARENB;        // Make 8n1
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;

		tty.c_cflag     &=  ~CRTSCTS;       // no flow control
		tty.c_cc[VMIN]      =   1;                  // read doesn't block
		tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
		tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

		/* Make raw */
		cfmakeraw(&tty);

		/* Flush Port, then applies attributes */
		tcflush(serd, TCIFLUSH);
		if(tcsetattr(serd, TCSANOW, &tty) != 0)
		{
			syslog(LOG_ERR, "Error %d from tcsetattr", errno);
		} else {
			serd_event = event_new(base, serd, EV_READ|EV_PERSIST, nrf_basenode_serd_cb_func, this);
		}
	}
	return rs;
}

void NRF_BASENODE::destroy_servant()
{
	if(serd_event) {
		event_free(serd_event);
	}
	ReServant::destroy_servant();
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
	//json2redislist();
	ReServant::loop();
}
