#include "reservant.h"
#include "hiredis_ext.h"

#include <stdio.h>
#include <linux/limits.h>
#include <math.h>
#include <syslog.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <adapters/libevent.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <event2/listener.h>
#include <event2/bufferevent.h>
#include <event2/buffer.h>

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
	sprintf(rs, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d.%6.6ld", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec, tv.tv_usec);
	return rs;
}

//#################
static void ReServant_on_udp_event(int fd, short int fields, void *arg)
{
	unsigned int unFromAddrLen;
	int nByte = 0;
	char aReqBuffer[512];
	struct sockaddr_in stFromAddr;

	unFromAddrLen = sizeof(stFromAddr);

	if ((nByte = recvfrom(fd, aReqBuffer, sizeof(aReqBuffer), 0, (struct sockaddr *)&stFromAddr, &unFromAddrLen)) == -1) {
		syslog(LOG_ERR, "error occured while receiving");
	} else {
		aReqBuffer[nByte] = 0;
		syslog(LOG_NOTICE, "Function called buffer is %s",aReqBuffer);
		ReServant *ths = (ReServant *)arg;
		ths->udp_request(stFromAddr, aReqBuffer);
	}

}

static void ReServant_tcp_read_cb(struct bufferevent *bev, void *ctx)
{
	ReServant *ths = (ReServant*)ctx;
	ths->tcp_request(bev);
}

static void ReServant_tcp_event_cb(struct bufferevent *bev, short events, void *ctx)
{
//	printf("Event:0x%2.2x\n", events);
	if (events & BEV_EVENT_ERROR) {
		syslog(LOG_ERR, "Error from bufferevent");
	}
//	if (events & (BEV_EVENT_EOF | BEV_EVENT_ERROR)) {
//		bufferevent_free(bev);
//	}
}

static void ReServant_accept_tcp_conn_cb(struct evconnlistener *listener, evutil_socket_t fd, struct sockaddr *address, int socklen, void *ctx)
{
	/* We got a new connection! Set up a bufferevent for it. */
	struct event_base *base = evconnlistener_get_base(listener);
	struct bufferevent *bev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
	bufferevent_setcb(bev, ReServant_tcp_read_cb, NULL, ReServant_tcp_event_cb, ctx);
	bufferevent_enable(bev, EV_READ|EV_WRITE);
}

static void ReServant_accept_tcp_error_cb(struct evconnlistener *listener, void *ctx)
{
	struct event_base *base = evconnlistener_get_base(listener);
	int err = EVUTIL_SOCKET_ERROR();
	syslog(LOG_ERR, "Got an error %d (%s) on the listener.", err, evutil_socket_error_to_string(err));
	event_base_loopexit(base, NULL);
}

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


ReServant::ReServant(const char *s_id):exiting(0),n_calls(0),cmdlist_size(0),timer_ev(NULL),servant_created(false),redis_list_size(DEFAULT_REDIS_LIST_SIZE)
{
	this->s_id = s_id;
	_mypath[readlink("/proc/self/exe", _mypath, sizeof(_mypath))] = 0;
	dirname(_mypath);
}

ReServant::~ReServant()
{
}

bool ReServant::create_servant()
{
	bool rs = false;
	if(!servant_created) {
		setlogmask (LOG_UPTO (LOG_DEBUG));
		static char logname[256];
		strncpy(logname, myid(), sizeof(logname));
		openlog(logname, LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
		char curpath[PATH_MAX+1], abscurpath[PATH_MAX+1];
		realpath(getcwd(curpath, sizeof(curpath)), curpath);
		syslog(LOG_NOTICE, "Hello from %s, path:%s\n", logname, curpath);
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

		setLoopInterval();
		redisLibeventAttach(aredis,base);
		redisAsyncSetConnectCallback(aredis, ReServant_connectCallback);
		redisAsyncSetDisconnectCallback(aredis, ReServant_disconnectCallback);
		redisAsyncCommand(aredis, ReServant_cmdCallback, this, "SUBSCRIBE %s", myid());
		servant_created = rs = true;
	}
	return rs;
}

void ReServant::destroy_servant()
{
	event_free(timer_ev);
	event_base_dispatch(base);
	closelog();
}

void ReServant::setCmdList(const pCMD_FUNC *cmdlist, int cmdlist_size)
{
	this->cmdlist = cmdlist;
	this->cmdlist_size = cmdlist_size;
}

void ReServant::setLoopInterval(float interval)
{
	struct timeval loop_timeout = { long(interval), long(interval * 1000000)%1000000  }; // 1.5 seconds
	syslog(LOG_NOTICE, "Loop interval set to %ld.%6.6ld seconds", loop_timeout.tv_sec, loop_timeout.tv_usec);
	if(timer_ev) {
		event_del(timer_ev);
	} else {
		timer_ev = event_new(base, -1, EV_PERSIST, ReServant_timer_cb_func, this);
	}
	event_add(timer_ev, &loop_timeout);
}

void ReServant::run()
{
	create_servant();
	do {
		event_base_loop(base, EVLOOP_NONBLOCK);
	} while(!exiting);
	destroy_servant();
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

int ReServant::processJsonCmd(json_t *js)
{
	int rs = -1;
	json_t *cmd = json_object_get(js, "cmd");
	for(int i=0; i< cmdlist_size; i++) {
		const pCMD_FUNC cf = cmdlist[i];
		if(strcmp(cf->cmd, json_string_value(cmd)) == 0) {
			rs = 0;
			call_cmd(cf, js);
			break;
		}
	}
	char *jstr = json_dumps(js, JSON_INDENT(4));
	if(jstr) {
		syslog(LOG_DEBUG, "%s\n", jstr);
		free(jstr);
	} else {
		syslog(LOG_ERR, "Can not encode JSON\n");
		rs = -2;
	}
	return rs;
}

void ReServant::cmdCallback(redisAsyncContext *c, redisReply *reply)
{
	//fprint_reply(reply);
	if(reply->type == REDIS_REPLY_ARRAY && reply->elements >= 3 && strcmp(reply->element[0]->str, "message") == 0) {
		printf("get %s\n", reply->element[2]->str);
		json_error_t error;
#ifdef JSON_DECODE_ANY
		json_t *js = json_loads(reply->element[2]->str, JSON_DECODE_ANY, &error);
#else
		json_t *js = json_loads(reply->element[2]->str, 0, &error);
#endif
		if (js == NULL) {
			syslog(LOG_ERR, "Error JSON decoding:%s", error.text);
		} else {
			processJsonCmd(js);
			json_decref(js);
		}
	}
}

void ReServant::runUDPserver(const char *host, int port)
{
	int udpsock_fd;
	if ((udpsock_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		syslog(LOG_ERR, "ERROR - unable to create socket");
		exit(1);
	}

	//Start : Set flags in non-blocking mode
	int nReqFlags = fcntl(udpsock_fd, F_GETFL, 0);
	if (nReqFlags< 0)
	{
		syslog(LOG_ERR, "ERROR - cannot set socket options");
	}

	if (fcntl(udpsock_fd, F_SETFL, nReqFlags | O_NONBLOCK) < 0)
	{
		syslog(LOG_ERR, "ERROR - cannot set socket options");
	}

	struct sockaddr_in stAddr;
	// End: Set flags in non-blocking mode
	memset(&stAddr, 0, sizeof(stAddr));
	if( host ) {
		stAddr.sin_addr.s_addr = inet_addr(host);
	} else {
		stAddr.sin_addr.s_addr = INADDR_ANY; //listening on local ip
	}
	stAddr.sin_port = htons(port);
	stAddr.sin_family = AF_INET;

	int nOptVal = 1;
	if (setsockopt(udpsock_fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&nOptVal, sizeof(nOptVal))) {
		syslog(LOG_ERR, "ERROR - socketOptions: Error at Setsockopt");
	}

	if (bind(udpsock_fd, (struct sockaddr *)&stAddr, sizeof(stAddr)) != 0)
	{
		syslog(LOG_ERR, "Error: Unable to bind the default IP");
		exit(-1);
	}

	syslog(LOG_NOTICE, "Listening UDP at port %d",  port);
	udp_ev = event_new(base, udpsock_fd, EV_READ | EV_PERSIST, ReServant_on_udp_event, this);
	event_add(udp_ev, NULL);

}

void ReServant::udp_request(sockaddr_in stFromAddr, const char *aReqBuffer)
{
}

void ReServant::runTCPserver(const char *host, int port)
{
	struct evconnlistener *listener;
	struct sockaddr_in stAddr;
	/* Clear the sockaddr before using it, in case there are extra
	/* platform-specific fields that can mess us up. */
	memset(&stAddr, 0, sizeof(stAddr));
	/* This is an INET address */
	stAddr.sin_family = AF_INET;
	/* Listen on 0.0.0.0 */
	if( host ) {
		stAddr.sin_addr.s_addr = inet_addr(host);
	} else {
		stAddr.sin_addr.s_addr = INADDR_ANY; //listening on local ip
	}
	/* Listen on the given port. */
	stAddr.sin_port = htons(port);
	listener = evconnlistener_new_bind(base, ReServant_accept_tcp_conn_cb, this, LEV_OPT_CLOSE_ON_FREE|LEV_OPT_REUSEABLE, -1, (struct sockaddr*)&stAddr, sizeof(stAddr));
	if (!listener) {
		syslog(LOG_ERR, "Couldn't create listener");
	}
	evconnlistener_set_error_cb(listener, ReServant_accept_tcp_error_cb);
	syslog(LOG_NOTICE, "Listening TCP at %s:%d",  (host) ? host : "", port);
}

void ReServant::tcp_request(struct bufferevent *bev)
{
}

void ReServant::runHttpd(const char *host, int port)
{
	/* Create a new evhttp object to handle requests. */
	http = evhttp_new(base);
	if (!http) {
		syslog(LOG_ERR, "couldn't create evhttp. Exiting.");
		exit(1);
	}

	/* The /dump URI will dump all requests to stdout and say 200 ok. */
	evhttp_set_cb(http, "/dump", ReServant_dump_request_cb, this);

	// The / URI callback
	evhttp_set_gencb(http, ReServant_cmd_request_cb, this);

	/* Now we tell the evhttp what port to listen on */
	sock = evhttp_bind_socket_with_handle(http, host, port);
	if (!sock) {
		syslog(LOG_ERR, "couldn't bind to port %d. Exiting.", (int)port);
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
		syslog(LOG_ERR, "Weird address family %d", ss.ss_family);
		exit(1);
	}
	addr = evutil_inet_ntop(ss.ss_family, inaddr, addrbuf, sizeof(addrbuf));
	if (addr) {
		syslog(LOG_NOTICE, "Listening on %s:%d", addr, got_port);
	} else {
		syslog(LOG_ERR, "evutil_inet_ntop failed");
		exit(1);
	}
}

void ReServant::http_request(struct evhttp_request *req)
{
}

bool ReServant::fill_json(json_t *js, int list_id)
{
	return false;
}

void ReServant::set_redis_list_limit(int val)
{
	redis_list_size = val;
	update_redis_list_size();
}

void ReServant::update_redis_list_size()
{
	redisAsyncCommand(aredis, NULL, NULL, "LTRIM %s.js.obj %d -1", myid(), -redis_list_size);
}

const char *ReServant::list_suffix(int list_id)
{
	return "";
}

void ReServant::json2redislist(int list_id)
{
	double t = dtime();
	json_t *js = json_object();
	json_object_set_new(js, "timestamp", json_real(t));
	if(fill_json(js, list_id)) {
		char *jstr = json_dumps(js, JSON_INDENT(4));
		if(jstr) {
			syslog(LOG_DEBUG, "%s\n", jstr);
			redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s%s.js.obj %s", myid(), list_suffix(list_id), jstr);
			update_redis_list_size();
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not encode JSON\n");
		}
	}
	json_decref(js);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.timestamp %s", myid(), s_timestamp(&t));
	redisAsyncCommand(aredis, NULL, NULL, "PUBLISH %s %d", myid(), list_id);
}

json_t *ReServant::tabbed2json(const char *ptr)
{
	char *tokensbuf = strdup(ptr);
	const char *delimiters = "\t:\xd";
	char *token;
	token = strtok(tokensbuf, delimiters);
	json_t *sjs = json_object();
	const char *key = NULL;
	while(token) {
		if(!key) {
			key = token;
		} else {
//			printf("%s => %s\n", key, token);
			json_object_set_new(sjs, key, json_real(atof(token)));
			key = NULL;
		}
		token = strtok(NULL, delimiters);
	}
	free(tokensbuf);
	return sjs;
}

