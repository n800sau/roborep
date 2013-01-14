#include <stdio.h>
#include <math.h>
#include <syslog.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>

#include <hiredis.h>
#include <async.h>
#include <adapters/libevent.h>

#include <jansson.h>


typedef void AFUNC(json_t *js);

struct CMD_FUNC {
	const char *cmd;
	AFUNC *func;
};


class RedisServant
{

	private:
		const char *r_cmd;
		const CMD_FUNC *cmdlist;
		const char *logname;

		int exiting;
		int n_calls;

		struct event_base *base;
		struct event *timer_ev;

		redisAsyncContext *aredis;
		redisContext *redis;

	protected:
		virtual void create_servant() = 0;
		void loop();

		static void cmdCallback(redisAsyncContext *c, void *r, void *privdata);
		static void connectCallback(const redisAsyncContext *c);
		static void disconnectCallback(const redisAsyncContext *c, int status);
		static void cb_func(evutil_socket_t fd, short what, void *privdata);

	public:

		RedisServant(const char *r_cmd, const CMD_FUNC *cmdlist, const char *logname);
		~RedisServant();

		void run();

};


RedisServant::RedisServant(const char *r_cmd, const CMD_FUNC *cmdlist, const char *logname):exiting(0),n_calls(0)
{
	this->logname = logname;
	this->r_cmd = r_cmd;
	this->cmdlist = cmdlist;
}

void RedisServant::run()
{
	setlogmask (LOG_UPTO (LOG_DEBUG));
	openlog(logname, LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);

	syslog(LOG_NOTICE, "Hello from %s\n", logname);
    signal(SIGPIPE, SIG_IGN);
    base = event_base_new();

	struct timeval timeout = { 1, 500000 }; // 1.5 seconds
	redis = redisConnectWithTimeout((char*)"localhost", 6379, timeout);
	if (redis->err) {
        syslog(LOG_ERR, "Connection error: %s\n", redis->errstr);
        exit(1);
    }
    aredis = redisAsyncConnect("localhost", 6379);
    if (aredis->err) {
        /* Let *aredis leak for now... */
        syslog(LOG_ERR, "Error: %s\n", aredis->errstr);
        exit(1);
    }

	create_servant();

    redisLibeventAttach(aredis,base);
    redisAsyncSetConnectCallback(aredis, connectCallback);
    redisAsyncSetDisconnectCallback(aredis, disconnectCallback);
    redisAsyncCommand(aredis, cmdCallback, this, "LPOP %s", r_cmd);
	timer_ev = event_new(base, -1, EV_PERSIST, cb_func, this);
	struct timeval one_sec = { 5, 0 };
	event_add(timer_ev, &one_sec);
	do {
    	event_base_loop(base, EVLOOP_NONBLOCK);
    } while(!exiting);
    event_base_dispatch(base);
	closelog();
}

void RedisServant::loop()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	redisAsyncCommand(aredis, NULL, NULL, "SET timestamp %d.%6.6d", tv.tv_sec, tv.tv_usec);
}

void RedisServant::connectCallback(const redisAsyncContext *c)
{
    ((void)c);
    syslog(LOG_NOTICE, "connected...\n");
}

void RedisServant::disconnectCallback(const redisAsyncContext *c, int status) {
    if (status != REDIS_OK) {
        syslog(LOG_ERR, c->errstr);
    }
    syslog(LOG_NOTICE, "disconnected...\n");
}

void RedisServant::cb_func(evutil_socket_t fd, short what, void *privdata)
{
	RedisServant *ths = (RedisServant *)privdata;
    syslog(LOG_NOTICE, "cb_func called %d times so far.\n", ++ths->n_calls);
	ths->loop();
}

void RedisServant::cmdCallback(redisAsyncContext *c, void *r, void *privdata)
{
	RedisServant *ths = (RedisServant *)privdata;
	redisReply *reply = (redisReply *)r;
	if (reply == NULL) {
		syslog(LOG_WARNING, "no reply\n");
	} else {
		if(reply->str) {
			json_error_t error;
			json_t *js = json_loads(reply->str, JSON_DECODE_ANY, &error);
			if (js == NULL) {
				syslog(LOG_ERR, "Error JSON decoding:%s", error.text);
			} else {
				json_t *cmd = json_object_get(js, "cmd");
				for(int i=0; i< sizeof(cmdlist) / sizeof(*cmdlist); i++) {
					const CMD_FUNC *cf = &ths->cmdlist[i];
					if(strcmp(cf->cmd, json_string_value(cmd)) == 0) {
						cf->func(js);
						break;
					}
				}
				char *jstr = json_dumps(js, JSON_INDENT(4));
				if(jstr) {
					syslog(LOG_DEBUG, "%s\n", jstr);
					free(jstr);
				} else {
					syslog(LOG_ERR, "Can not decode JSON\n");
				}
				json_decref(js);
			}
		}
	}
	redisAsyncCommand(ths->aredis, cmdCallback, NULL, "LPOP %s", ths->r_cmd);
}

