#include "HD44780.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <ifaddrs.h>
#include <errno.h>
#include <arpa/inet.h>

struct HD44780_CMD_FUNC:public CMD_FUNC {
	public:
		HD44780_CMD_FUNC(const char *cmd, HD44780::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		HD44780::tFunction ptr;
};

typedef HD44780_CMD_FUNC *pHD44780_CMD_FUNC;

HD44780::HD44780():ReServant("hd44780"),listpath(NULL)
{
	const static pHD44780_CMD_FUNC cmdlist[] = {
		new HD44780_CMD_FUNC("stop_state", &HD44780::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

HD44780::~HD44780()
{
	setListPath(NULL);
}

void HD44780::setListPath(const char *listpath)
{
	if(this->listpath) {
		free((void*)this->listpath);
	}
	if(listpath) {
		this->listpath = strdup(listpath);
	} else {
		this->listpath = NULL;
	}
}

void HD44780::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pHD44780_CMD_FUNC ccmd = (pHD44780_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void HD44780::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
}

bool HD44780::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		setLoopInterval(5);
		if(listpath) {
			syslog(LOG_INFO, "Listfile=%s", listpath);
		}
		/* initialise HD44780 */
		//specifies the pins that will be used
		int selectedPins[]={P8_14,P8_12,P8_11,P8_5,P8_4,P8_3};

	    initialize_Screen(enabled_gpio,selectedPins);	

		//clear screen
		clear_Screen(enabled_gpio);
	}
	return rs;
}

int filterIf(const char *ifname, char **ifbuf, int nbuf)
{
	int rs = -1;
	for(int i=0; i<nbuf; i++) {
//		printf("Comparing %s with %s = %d\n", ifbuf[i], ifname, strcmp(ifbuf[i], ifname));
		if(strcmp(ifbuf[i], ifname) == 0) {
			rs = i;
			break;
		}
	}
	return rs;
}

void HD44780::loop()
{
	clear_Screen(enabled_gpio);
	FILE *f = (listpath) ? fopen(listpath, "r") : NULL;
	if(f) {
		const int nlines = 2;
		char ifbuf[nlines][20];
		char *pifbuf[nlines];
		int n = 0;
		while(fgets(ifbuf[n], sizeof(ifbuf[n]), f)) {
			pifbuf[n] = ifbuf[n];
			int l = strlen(ifbuf[n]) - 1;
			if(ifbuf[n][l] == '\n') {
				ifbuf[n][l] = 0;
			}
			if(++n >= nlines) break;
		}
		fclose(f);
		if(n > 0) {
			struct ifaddrs *ifap, *ifa;
			if(getifaddrs(&ifap) == 0) {
				for (ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
					if(ifa ->ifa_addr->sa_family==AF_INET) { // check it is IP4
						int pos = filterIf(ifa->ifa_name, pifbuf, n);
						if(pos >= 0) {
							// is a valid IP4 Address
							struct in_addr *tmpAddrPtr;
							tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
							char addressBuffer[INET_ADDRSTRLEN];
							inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
							syslog(LOG_INFO, "%s IP Address %s\n", ifa->ifa_name, addressBuffer);
							//go to the the line
							goto_ScreenLine(pos,enabled_gpio);
							stringToScreen(addressBuffer,enabled_gpio);
						}
/*					} else if (ifa->ifa_addr->sa_family==AF_INET6) { // check it is IP6
						if(filterIf(ifa->ifa_name, pifbuf, n)) {
							// is a valid IP6 Address
							struct in6_addr *tmpAddrPtr;
							tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
							char addressBuffer[INET6_ADDRSTRLEN];
							inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
							syslog(LOG_INFO, "%s IP6 Address %s\n", ifa->ifa_name, addressBuffer);
						}*/
					}
				}
				freeifaddrs(ifap);
			} else {
				syslog(LOG_ERR, "Reading network interfaces error: %s", strerror(errno));
			}
		}
	}
	ReServant::loop();
	
}
