#ifndef __RESERVANT_H

#define __RESERVANT_H

#include <jansson.h>
#include <hiredis.h>
#include <async.h>



class ReServant
{
	protected:

		typedef void AFUNC(json_t *js);

		struct CMD_FUNC {
			const char *cmd;
			AFUNC *func;
		};


	private:
		const char *r_cmd;
		const CMD_FUNC *cmdlist;
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

	public:

		ReServant(const char *r_cmd, const char *logname);
		~ReServant();

		void setCmdList(const CMD_FUNC *cmdlist);
		void run();

		void cb_func(short what);
		void cmdCallback(redisAsyncContext *c, redisReply *reply);
};


#endif //__RESERVANT_H
