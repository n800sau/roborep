#ifndef __ATEXEC_H
#define __ATEXEC_H

#include <WiFiClient.h>
#include <LimitedList.h>


class AtExec {
	protected:
		WiFiClient client;
		LimitedList<String, 5> argv;
	public:
		inline AtExec(WiFiClient client):
			client(client),argv(false)
			{}

		void parseCommand(String &cmd);

		static void print_help(AtExec *self);
		static void config_baud(AtExec *self);
};

#endif //__ATEXEC_H
