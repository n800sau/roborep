#ifndef __ATEXEC_H
#define __ATEXEC_H

#include "TelClient.h"
#include <LimitedList.h>


class AtExec {
	protected:
		TelClient *client;
		LimitedList<String, 5> argv;
	public:
		AtExec(TelClient *client);

		bool parseCommand(String &cmd);
		void ok();
		void error();

		static void print_help(AtExec *self);
		static void config_baud(AtExec *self);
		static void cmd_reset(AtExec *self);
		static void io_reset(AtExec *self);
		static void show_info(AtExec *self);
		static void show_scan(AtExec *self);
		static void config_mode(AtExec *self);
		static void config_ap(AtExec *self);
		static void config_sta(AtExec *self);
		static void config_debug_mode(AtExec *self);
		static void unknown_command(AtExec *self);
};

#endif //__ATEXEC_H
