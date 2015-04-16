#ifndef __TELCLIENT_H
#define __TELCLIENT_H

#include <WiFiClient.h>

class TelClient {

	public:

		TelClient();

		WiFiClient conn;
		String remains;
		String dbgBuff;
		bool dbgMode;
};

#endif //__TELCLIENT_H
