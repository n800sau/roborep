#include "pc2nrf.h"
#include <iostream>
#include <exception>
#include <errno.h>
#include <syslog.h>

int main(int argc, char const **argv)
{
	try {
		PC2NRF srv = PC2NRF();
		srv.run();
		return 0;
	} catch (const std::exception& e) {
		char buf[256];
		snprintf(buf, sizeof(buf), "Error %s\n%s\n", e.what(), strerror(errno));
		syslog(LOG_ERR, buf);
		fputs(buf, stderr);
		return 1;
	}
}
