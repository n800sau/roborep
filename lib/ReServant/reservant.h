#ifndef __RESERVANT_H

#define __RESERVANT_H

#include <jansson.h>
#include <hiredis.h>
#include <async.h>

struct CMD_FUNC {
	const char *cmd;
};

typedef CMD_FUNC *pCMD_FUNC;

class ReServant
{

	private:
		const char *r_cmd;
		const pCMD_FUNC *cmdlist;
		int cmdlist_size;
		const char *logname;

		int exiting;
		int n_calls;

		struct event_base *base;
		struct event *timer_ev;

	protected:

		redisAsyncContext *aredis;
		redisContext *redis;

		virtual void create_servant();
		virtual void loop();

		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);

	public:

		ReServant(const char *r_cmd, const char *logname);
		~ReServant();

		void setCmdList(const pCMD_FUNC *cmdlist, int cmdlist_size);
		void run();

		void cb_func(short what);
		void cmdCallback(redisAsyncContext *c, redisReply *reply);
};


#endif //__RESERVANT_H
