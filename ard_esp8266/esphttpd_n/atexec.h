#include <WiFiClient.h>

class AtExec {
	protected:
		WiFiClient client;
	public:
		inline AtExec(WiFiClient client): client(client) {}

		void parseCommand(String &cmd);
};
