#ifndef __RESERVANT_H

#define __RESERVANT_H

#include <jansson.h>
#include <hiredis.h>
#include <async.h>
#include <event2/http.h>
#include <time.h>
#include <event2/bufferevent.h>

#define REDIS_LIST_SIZE 10


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

		struct event *timer_ev;
		struct event *udp_ev;

		struct evhttp *http;
		struct evhttp_bound_socket *sock;

		char _mypath[256];

		bool servant_created;

	protected:

		struct event_base *base;

		redisAsyncContext *aredis;
		redisContext *redis;

		virtual void loop();

		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);

		inline const char *mypath() { return _mypath; }

		inline const char *myid() { return s_id; }

		virtual void fill_json(json_t *js);

		void json2redislist();

		int processJsonCmd(json_t *js);

	public:

		ReServant(const char *s_id);
		~ReServant();

		void setCmdList(const pCMD_FUNC *cmdlist, int cmdlist_size);
		void run();
		//can be run after run() only
		void runHttpd(const char *host, int port);
		//can be run after run() only
		void runUDPserver(const char *host, int port);
		//can be run after run() only
		void runTCPserver(const char *host, int port);

		void cmdCallback(redisAsyncContext *c, redisReply *reply);
		void timer_cb_func(short what);

		virtual void http_request(struct evhttp_request *req);
		virtual void udp_request(sockaddr_in stFromAddr, const char *aReqBuffer);
		virtual void tcp_request(struct bufferevent *bev);
		virtual bool create_servant();

		//loop interval in seconds
		void setLoopInterval(float interval=0.5);
};

double dtime();
const char *s_timestamp(const double *dt=NULL);

#endif //__RESERVANT_H