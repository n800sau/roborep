#include "tcpserver.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <time.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <arpa/inet.h>

struct USERVER_CMD_FUNC:public CMD_FUNC {
	public:
		USERVER_CMD_FUNC(const char *cmd, TCPServer::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		TCPServer::tFunction ptr;
};

typedef USERVER_CMD_FUNC *pUSERVER_CMD_FUNC;


tcpserver_redisReply::tcpserver_redisReply(TCPServer *ths, const char *rgroup, const char *rtype, const char *rkey)
{
	this->ths = ths;
	this->rgroup = strdup(rgroup);
	this->rtype = strdup(rtype);
	this->rkey = (rkey) ? strdup(rkey) : NULL; 
}

tcpserver_redisReply::~tcpserver_redisReply()
{
	free((void*)this->rgroup);
	free((void*)this->rtype);
	if(this->rkey)
		free((void*)this->rkey);
}

void TCPServer_keyCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	tcpserver_redisReply *rr = (tcpserver_redisReply *)privdata;
	rr->ths->keyCallback(c, rr->rgroup, rr->rtype, (redisReply *)r);
	delete rr;
}

void TCPServer_paramCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	tcpserver_redisReply *rr = (tcpserver_redisReply *)privdata;
	rr->ths->paramCallback(c, rr->rkey, rr->rgroup, rr->rtype, (redisReply *)r);
	delete rr;
}

void TCPServer_send_full_cb_func(evutil_socket_t fd, short what, void *privdata)
{
	SEND_FULL_DATA *d = (SEND_FULL_DATA *)privdata;
	d->ths->send_full_cb_func(d);
}


TCPServer::TCPServer(int port):ReServant("tcpserver"),port(port),rvals_count(0)
{
	memset(bevs, 0, sizeof(bevs));
	const static pUSERVER_CMD_FUNC cmdlist[] = {
		new USERVER_CMD_FUNC("send_full_data", &TCPServer::send_full_data),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

TCPServer::~TCPServer()
{
	for(int i=0; i<rvals_count; i++) {
		delete rvals[i];
	}
}

bool TCPServer::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		printf("Running...\n");
		runTCPserver(NULL, port);
	}
	return rs;
}

void TCPServer::loop()
{
//	printf("............................................\n");
	for(int i=0; i<rgroups_count; i++) {
		const char *rgroup = rgroups[i];
		tcpserver_redisReply *rr;
		rr = new tcpserver_redisReply(this, rgroup, "r");
		redisAsyncCommand(aredis, TCPServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new tcpserver_redisReply(this, rgroup, "i");
		redisAsyncCommand(aredis, TCPServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new tcpserver_redisReply(this, rgroup, "s");
		redisAsyncCommand(aredis, TCPServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new tcpserver_redisReply(this, rgroup, "js");
		redisAsyncCommand(aredis, TCPServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
	}
	ReServant::loop();
}

bool TCPServer::add_bev(struct bufferevent *bev, json_t *js)
{
	bool rs = false;
	for(int i=0; i<MAX_BUFEV_COUNT; i++) {
		if(!bevs[i]) {
			bevs[i] = bev;
			json_object_set(js, "bev_index", json_integer(i));
			rs = true;
			break;
		}
	}
	return rs;
}

void TCPServer::remove_bev(struct bufferevent *bev)
{
	for(int i=0; i<MAX_BUFEV_COUNT; i++) {
		if(bevs[i] == bev) {
			bufferevent_free(bev);
			bevs[i] = NULL;
			break;
		}
	}
}

void TCPServer::tcp_request(struct bufferevent *bev)
{
	/* This callback is invoked when there is data to read on bev. */
	struct evbuffer *input = bufferevent_get_input(bev);
	bool do_free = true;
	int ilen = evbuffer_get_length(input);
	char *buf = (char*)malloc(ilen+1);
	ilen = evbuffer_copyout(input, buf, ilen+1);
	buf[ilen] = 0;
	json_error_t error;
	json_t *js = json_loads(buf, 0, &error);
	if(add_bev(bev, js)) {
		if(js == NULL) {
			syslog(LOG_ERR, "Error decoding json %s\n%s", buf, error.text);
		} else {
			if(processJsonCmd(js) < 0) {
				syslog(LOG_ERR, "Error processing json command");
			} else {
				do_free = false;
			}
		}
		if(do_free) {
			remove_bev(bev);
		}
	} else {
		syslog(LOG_ERR, "No more space for bufferevents");
		bufferevent_free(bev);
	}
	free((void*)buf);
}

void TCPServer::keyCallback(redisAsyncContext *c, const char *rgroup, const char *rtype, redisReply *reply)
{
	if (reply == NULL) {
		syslog(LOG_WARNING, "no keys in reply\n");
	} else {
		if(reply->type == REDIS_REPLY_ARRAY) {
//			printf(">>>%s.%s: %d\n", rgroup, rtype, reply->elements);
			for(int i=0;i<reply->elements; i++) {
				redisReply *el = reply->element[i];
//				printf("key reply %s\n", el->str);
				tcpserver_redisReply *rr = new tcpserver_redisReply(this, rgroup, rtype, el->str);
				if ( strcmp(rr->rtype, rtypes[R_JSON]) == 0 ) {
					redisAsyncCommand(aredis, TCPServer_paramCallBack, rr, "LINDEX %s -1", rr->rkey);
				} else {
					redisAsyncCommand(aredis, TCPServer_paramCallBack, rr, "GET %s", rr->rkey);
				}
			}
		}
	}
}

void TCPServer::paramCallback(redisAsyncContext *c, const char *rkey, const char *rgroup, const char *rtype, redisReply *reply)
{
	if (reply == NULL) {
		syslog(LOG_WARNING, "no reply\n");
	} else {
		if(reply->str) {
			int found=-1;
			for(int i=0; i<rvals_count; i++) {
				if(strcmp(rvals[i]->key, rkey) == 0) {
					found = i;
					break;
				}
			}
			if(found < 0) {
				if (rvals_count < MAX_RVALS_COUNT) {
					found = rvals_count;
					rvals_count++;
				}
			} else {
				delete rvals[found];
			}
			if (found >= 0) {
				if ( strcmp(rtype, rtypes[R_REAL]) == 0 ) {
					rvals[found] = new RVAL(rkey, atof(reply->str));
//					printf("param reply real %s=%g\n", rvals[found]->key, rvals[found]->u.rval);
				} else if ( strcmp(rtype, rtypes[R_INT]) == 0 ) {
					rvals[found] = new RVAL(rkey, atoi(reply->str));
//					printf("param reply int %s=%d\n", rvals[found]->key, rvals[found]->u.ival);
				} else if ( strcmp(rtype, rtypes[R_STR]) == 0 ) {
					rvals[found] = new RVAL(rkey, reply->str);
//					printf("param reply str %s=%s\n", rvals[found]->key, rvals[found]->u.sval);
				} else if ( strcmp(rtype, rtypes[R_JSON]) == 0 ) {
					rvals[found] = new RVAL(rkey, reply->str, R_JSON);
//					printf("param reply str %s=%s\n", rvals[found]->key, rvals[found]->u.sval);
				}
			}
		}
	}
}

void TCPServer::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pUSERVER_CMD_FUNC ccmd = (pUSERVER_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void TCPServer::send_full_data(json_t *js)
{
	json_t *js_v;
	long interval = json_integer_value(json_object_get(js, "interval"));
	if(interval <= 0)
		interval = 1000;
	int count = json_integer_value(json_object_get(js, "count"));
	if(count <= 0)
		count = 1;
	struct timeval loop_timeout = { interval/1000, (interval%1000)*1000  };
	SEND_FULL_DATA *data = new SEND_FULL_DATA(this, js, count);
	data->timer_ev = event_new(base, -1, EV_PERSIST, TCPServer_send_full_cb_func, data);
	event_add(data->timer_ev, &loop_timeout);
}

void TCPServer::send_full_cb_func(SEND_FULL_DATA *d)
{
	struct bufferevent *bev = bevs[json_integer_value(json_object_get(d->js, "bev_index"))];
	if(d->count > 0) {
		syslog(LOG_NOTICE, "Sending time %d", d->count);
		d->count--;
		json_t *js = json_object();
		json_object_set_new(js, "mypath", json_string(mypath()));
		json_object_set_new(js, "s_timestamp", json_string(s_timestamp()));
		for(int i=0; i<rvals_count; i++) {
			switch(rvals[i]->rtype) {
				case R_INT:
					json_object_set_new(js, rvals[i]->key, json_integer(rvals[i]->u.ival));
					break;
				case R_REAL:
					json_object_set_new(js, rvals[i]->key, json_real(rvals[i]->u.rval));
					break;
				case R_STR:
					json_object_set_new(js, rvals[i]->key, json_string(rvals[i]->u.sval));
					break;
				case R_JSON:
					json_error_t error;
					json_t *jsv = json_loads(rvals[i]->u.sval, JSON_DISABLE_EOF_CHECK, &error);
					json_object_set_new(js, rvals[i]->key, jsv);
					break;
			}
		}
		char *jstr = json_dumps(js, JSON_COMPACT);
//		char *jstr = json_dumps(js, JSON_INDENT(4));
		if(jstr) {
//			syslog(LOG_NOTICE, "Sending using %p ...", bevs[json_integer_value(json_object_get(d->js, "bev_index"))]);
			bufferevent_write(bev, jstr, strlen(jstr));
			bufferevent_write(bev, "\n", strlen("\n"));
//			const char eof[] = "\nEOF\n";
//			bufferevent_write(bev, eof, strlen(eof));
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not decode JSON");
		}
		json_decref(js);
	} else {
		remove_bev(bev);
		event_del(d->timer_ev);
		delete d;
	}
}
