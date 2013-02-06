#ifndef __RESERVANT_H

#define __RESERVANT_H

#include <jansson.h>
#include <hiredis.h>
#include <async.h>
#include <event2/http.h>


struct CMD_FUNC {
	const char *cmd;
};

typedef CMD_FUNC *pCMD_FUNC;

class ReServant
{

	private:
		const char *s_id;
		const pCMD_FUNC *cmdlist;
		int cmdlist_size;

		int exiting;
		int n_calls;

		struct event_base *base;
		struct event *timer_ev;

		struct evhttp *http;
		struct evhttp_bound_socket *sock;

		char _mypath[256];
	protected:

		redisAsyncContext *aredis;
		redisContext *redis;

		virtual void create_servant();
		virtual void loop();

		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);

		inline const char *mypath() { return _mypath; }

		inline const char *myid() { return s_id; }

	public:

		ReServant(const char *s_id);
		~ReServant();

		void setCmdList(const pCMD_FUNC *cmdlist, int cmdlist_size);
		void run();
		//can be run after run() only
		void runHttpd(const char *host, int port);

		void cmdCallback(redisAsyncContext *c, redisReply *reply);
		void timer_cb_func(short what);

		virtual void http_request(struct evhttp_request *req);
};


#endif //__RESERVANT_H
