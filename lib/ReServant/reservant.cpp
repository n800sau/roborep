#include "reservant.h"

#include <stdio.h>
#include <math.h>
#include <syslog.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <adapters/libevent.h>
#include <unistd.h>
#include <libgen.h>


double dtime()
{
	timeval rs;
	gettimeofday(&rs, NULL);
	return double(rs.tv_usec) / 1000000 + rs.tv_sec;
}

const char *s_timestamp(const double *dt)
{
	timeval tv;
	static char rs[50];
	if(dt) {
		tv.tv_sec = (time_t)*dt;
		tv.tv_usec = (suseconds_t)(long(((*dt) * 1000000)) % 1000000);
	} else {
		gettimeofday(&tv, NULL);
	}
	struct tm *st = localtime(&tv.tv_sec);
	sprintf(rs, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d.%6.6d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec, tv.tv_usec);
	return rs;
}


//#################
static void ReServant_cmd_request_cb(struct evhttp_request *req, void *arg)
{
	ReServant *ths = (ReServant *)arg;
	ths->http_request(req);
}

static void ReServant_dump_request_cb(struct evhttp_request *req, void *arg)
{
	const char *cmdtype;
	struct evkeyvalq *headers;
	struct evkeyval *header;
	struct evbuffer *buf;

	switch (evhttp_request_get_command(req)) {
		case EVHTTP_REQ_GET: cmdtype = "GET"; break;
		case EVHTTP_REQ_POST: cmdtype = "POST"; break;
		case EVHTTP_REQ_HEAD: cmdtype = "HEAD"; break;
		case EVHTTP_REQ_PUT: cmdtype = "PUT"; break;
		case EVHTTP_REQ_DELETE: cmdtype = "DELETE"; break;
		case EVHTTP_REQ_OPTIONS: cmdtype = "OPTIONS"; break;
		case EVHTTP_REQ_TRACE: cmdtype = "TRACE"; break;
		case EVHTTP_REQ_CONNECT: cmdtype = "CONNECT"; break;
		case EVHTTP_REQ_PATCH: cmdtype = "PATCH"; break;
		default: cmdtype = "unknown"; break;
	}

	syslog(LOG_NOTICE, "Received a %s request for %s\nHeaders:\n", cmdtype, evhttp_request_get_uri(req));

	headers = evhttp_request_get_input_headers(req);
	for (header = headers->tqh_first; header;
	    header = header->next.tqe_next) {
		syslog(LOG_NOTICE, "  %s: %s\n", header->key, header->value);
	}

	buf = evhttp_request_get_input_buffer(req);
	syslog(LOG_NOTICE, "Input data: <<<");
	while (evbuffer_get_length(buf)) {
		int n;
		char cbuf[128];
		n = evbuffer_remove(buf, cbuf, sizeof(buf)-1);
		if (n > 0)
			syslog(LOG_NOTICE, "%s", cbuf);
	}
	syslog(LOG_NOTICE, ">>>");

	evhttp_send_reply(req, 200, "OK", NULL);
}

void ReServant_connectCallback(const redisAsyncContext *c)
{
    ((void)c);
    syslog(LOG_NOTICE, "connected...\n");
}

void ReServant_disconnectCallback(const redisAsyncContext *c, int status) {
    if (status != REDIS_OK) {
        syslog(LOG_ERR, "%s while disconnecting", c->errstr);
    }
    syslog(LOG_NOTICE, "disconnected...\n");
}

void ReServant_timer_cb_func(evutil_socket_t fd, short what, void *privdata)
{
	ReServant *ths = (ReServant *)privdata;
	ths->timer_cb_func(what);
}

void ReServant_cmdCallback(redisAsyncContext *c, void *r, void *privdata)
{
	ReServant *ths = (ReServant *)privdata;
	ths->cmdCallback(c, (redisReply *)r);
}
//###############


ReServant::ReServant(const char *s_id):exiting(0),n_calls(0),cmdlist_size(0)
{
	this->s_id = s_id;
	_mypath[readlink("/proc/self/exe", _mypath, sizeof(_mypath))] = 0;
	dirname(_mypath);
}

ReServant::~ReServant()
{
}

void ReServant::create_servant()
{
}

void ReServant::setCmdList(const pCMD_FUNC *cmdlist, int cmdlist_size)
{
	this->cmdlist = cmdlist;
	this->cmdlist_size = cmdlist_size;
}

void ReServant::run()
{
	setlogmask (LOG_UPTO (LOG_DEBUG));
	char logname[256];
	sprintf(logname, "%s", myid());
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
    redisAsyncSetConnectCallback(aredis, ReServant_connectCallback);
    redisAsyncSetDisconnectCallback(aredis, ReServant_disconnectCallback);
    redisAsyncCommand(aredis, ReServant_cmdCallback, this, "LPOP cmd.%s", myid());
	timer_ev = event_new(base, -1, EV_PERSIST, ReServant_timer_cb_func, this);
	struct timeval one_sec = { 0, 500000 };
	event_add(timer_ev, &one_sec);
	do {
    	event_base_loop(base, EVLOOP_NONBLOCK);
    } while(!exiting);
    event_base_dispatch(base);
	closelog();
}

void ReServant::loop()
{
}

void ReServant::timer_cb_func(short what)
{
//    syslog(LOG_NOTICE, "timer_cb_func called %d times so far.\n", ++n_calls);
	loop();
}

void ReServant::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
}

void ReServant::cmdCallback(redisAsyncContext *c, redisReply *reply)
{
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
				for(int i=0; i< cmdlist_size; i++) {
					const pCMD_FUNC cf = cmdlist[i];
					if(strcmp(cf->cmd, json_string_value(cmd)) == 0) {
						call_cmd(cf, js);
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
	redisAsyncCommand(aredis, ReServant_cmdCallback, this, "LPOP cmd.%s", myid());
}

void ReServant::http_request(struct evhttp_request *req)
{
}

void ReServant::runHttpd(const char *host, int port)
{
	/* Create a new evhttp object to handle requests. */
	http = evhttp_new(base);
	if (!http) {
		syslog(LOG_ERR, "couldn't create evhttp. Exiting.\n");
		exit(1);
	}

	/* The /dump URI will dump all requests to stdout and say 200 ok. */
	evhttp_set_cb(http, "/dump", ReServant_dump_request_cb, this);

	// The / URI callback
	evhttp_set_gencb(http, ReServant_cmd_request_cb, this);

	/* Now we tell the evhttp what port to listen on */
	sock = evhttp_bind_socket_with_handle(http, host, port);
	if (!sock) {
		syslog(LOG_ERR, "couldn't bind to port %d. Exiting.\n", (int)port);
		exit(1);
	}
	/* Extract and display the address we're listening on. */
	struct sockaddr_storage ss;
	evutil_socket_t fd;
	ev_socklen_t socklen = sizeof(ss);
	char addrbuf[128];
	void *inaddr;
	const char *addr;
	int got_port = -1;
	fd = evhttp_bound_socket_get_fd(sock);
	memset(&ss, 0, sizeof(ss));
	if (getsockname(fd, (struct sockaddr *)&ss, &socklen)) {
		syslog(LOG_ERR, "getsockname() failed");
		exit(1);
	}
	if (ss.ss_family == AF_INET) {
		got_port = ntohs(((struct sockaddr_in*)&ss)->sin_port);
		inaddr = &((struct sockaddr_in*)&ss)->sin_addr;
	} else if (ss.ss_family == AF_INET6) {
		got_port = ntohs(((struct sockaddr_in6*)&ss)->sin6_port);
		inaddr = &((struct sockaddr_in6*)&ss)->sin6_addr;
	} else {
		syslog(LOG_ERR, "Weird address family %d\n", ss.ss_family);
		exit(1);
	}
	addr = evutil_inet_ntop(ss.ss_family, inaddr, addrbuf, sizeof(addrbuf));
	if (addr) {
		syslog(LOG_NOTICE, "Listening on %s:%d\n", addr, got_port);
	} else {
		syslog(LOG_ERR, "evutil_inet_ntop failed\n");
		exit(1);
	}
}

void ReServant::fill_json(json_t *js)
{
}

void ReServant::json2redislist()
{
	double t = dtime();
	json_t *js = json_object();
	json_object_set_new(js, "timestamp", json_real(t));
	fill_json(js);
	char *jstr = json_dumps(js, JSON_INDENT(4));
	if(jstr) {
//		syslog(LOG_DEBUG, "%s\n", jstr);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s.js.obj %s", myid(), jstr);
		redisAsyncCommand(aredis, NULL, NULL, "LTRIM %s.js.obj %d -1", myid(), -REDIS_LIST_SIZE-1);
		free(jstr);
	} else {
		syslog(LOG_ERR, "Can not encode JSON\n");
	}
	json_decref(js);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.timestamp %s", myid(), s_timestamp(&t));
}

