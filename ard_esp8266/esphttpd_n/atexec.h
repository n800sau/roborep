#ifndef __ATEXEC_H
#define __ATEXEC_H

#include <WiFiClient.h>
#include <LimitedList.h>


class AtExec {
	protected:
		WiFiClient client;
		LimitedList<String, 5> argv;
	public:
		AtExec(WiFiClient client);

		bool parseCommand(String &cmd);
		void ok();
		void error();

		static void print_help(AtExec *self);
		static void config_baud(AtExec *self);
		static void cmd_reset(AtExec *self);
		static void io_reset(AtExec *self);
		static void show_netinfo(AtExec *self);
		static void config_cmd_mode(AtExec *self);
		static void unknown_command(AtExec *self);
};

#endif //__ATEXEC_H
