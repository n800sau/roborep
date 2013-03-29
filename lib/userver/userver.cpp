#include "userver.h"
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
		USERVER_CMD_FUNC(const char *cmd, UServer::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		UServer::tFunction ptr;
};

typedef USERVER_CMD_FUNC *pUSERVER_CMD_FUNC;


userver_redisReply::userver_redisReply(UServer *ths, const char *rgroup, const char *rtype, const char *rkey)
{
	this->ths = ths;
	this->rgroup = strdup(rgroup);
	this->rtype = strdup(rtype);
	this->rkey = (rkey) ? strdup(rkey) : NULL; 
}

userver_redisReply::~userver_redisReply()
{
	free((void*)this->rgroup);
	free((void*)this->rtype);
	if(this->rkey)
		free((void*)this->rkey);
}

void UServer_keyCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	userver_redisReply *rr = (userver_redisReply *)privdata;
	rr->ths->keyCallback(c, rr->rgroup, rr->rtype, (redisReply *)r);
	delete rr;
}

void UServer_paramCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	userver_redisReply *rr = (userver_redisReply *)privdata;
	rr->ths->paramCallback(c, rr->rkey, rr->rgroup, rr->rtype, (redisReply *)r);
	delete rr;
}

struct SEND_FULL_DATA {
	UServer *ths;
	json_t *js;
	int count;
	struct event *timer_ev;
	inline SEND_FULL_DATA(UServer *ths, json_t *js, int count)
	{
		this->js = js;
		this->ths = ths;
		this->count = count;
	}
};

void UServer_send_full_cb_func(evutil_socket_t fd, short what, void *privdata)
{
	SEND_FULL_DATA *d = (SEND_FULL_DATA *)privdata;
	if(d->count > 0) {
		syslog(LOG_NOTICE, "Sending time %d", d->count);
		d->ths->send_full_cb_func(d->js);
		d->count--;
	} else {
		event_del(d->timer_ev);
		delete d;
	}
}


UServer::UServer(int port):ReServant("userver"),port(port),rvals_count(0)
{
	const static pUSERVER_CMD_FUNC cmdlist[] = {
		new USERVER_CMD_FUNC("send_full_data", &UServer::send_full_data),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

UServer::~UServer()
{
	for(int i=0; i<rvals_count; i++) {
		delete rvals[i];
	}
}

bool UServer::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		printf("Running...\n");
		runUDPserver("0.0.0.0", port);
	}
	return rs;
}

void UServer::loop()
{
//	printf("............................................\n");
	for(int i=0; i<rgroups_count; i++) {
		const char *rgroup = rgroups[i];
		userver_redisReply *rr;
		rr = new userver_redisReply(this, rgroup, "r");
		redisAsyncCommand(aredis, UServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new userver_redisReply(this, rgroup, "i");
		redisAsyncCommand(aredis, UServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new userver_redisReply(this, rgroup, "s");
		redisAsyncCommand(aredis, UServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
		rr = new userver_redisReply(this, rgroup, "js");
		redisAsyncCommand(aredis, UServer_keyCallBack, rr, "KEYS %s.%s.*", rr->rgroup, rr->rtype);
	}
	ReServant::loop();
}

void UServer::udp_request(sockaddr_in stFromAddr, const char *aReqBuffer)
{
	char hbuf[NI_MAXHOST], sbuf[NI_MAXSERV];
	if (getnameinfo((sockaddr*)&stFromAddr, sizeof(stFromAddr), hbuf, sizeof(hbuf), sbuf, sizeof(sbuf), NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
		printf("%s came from %s:%s\n", aReqBuffer, hbuf, sbuf);
		json_error_t error;
		json_t *js = json_loads(aReqBuffer, JSON_DECODE_ANY, &error);
		if(js == NULL) {
			printf("Error decoding json %s\n", error.text);
		} else {
			json_object_set(js, "src_host", json_string(hbuf));
			json_object_set(js, "src_port", json_integer(atol(sbuf)));
			if(processJsonCmd(js) < 0) {
				printf("Error processing json command\n");
			}
		}
	} else {
		printf("Error getting source address\n");
	}
}

void UServer::keyCallback(redisAsyncContext *c, const char *rgroup, const char *rtype, redisReply *reply)
{
	if (reply == NULL) {
		syslog(LOG_WARNING, "no keys in reply\n");
	} else {
		if(reply->type == REDIS_REPLY_ARRAY) {
//			printf(">>>%s.%s: %d\n", rgroup, rtype, reply->elements);
			for(int i=0;i<reply->elements; i++) {
				redisReply *el = reply->element[i];
//				printf("key reply %s\n", el->str);
				userver_redisReply *rr = new userver_redisReply(this, rgroup, rtype, el->str);
				if ( strcmp(rr->rtype, rtypes[R_JSON]) == 0 ) {
					redisAsyncCommand(aredis, UServer_paramCallBack, rr, "LINDEX %s -1", rr->rkey);
				} else {
					redisAsyncCommand(aredis, UServer_paramCallBack, rr, "GET %s", rr->rkey);
				}
			}
		}
	}
}

void UServer::paramCallback(redisAsyncContext *c, const char *rkey, const char *rgroup, const char *rtype, redisReply *reply)
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

void UServer::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pUSERVER_CMD_FUNC ccmd = (pUSERVER_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void UServer::send_full_data(json_t *js)
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
	data->timer_ev = event_new(base, -1, EV_PERSIST, UServer_send_full_cb_func, data);
	event_add(data->timer_ev, &loop_timeout);
}

void UServer::send_full_cb_func(json_t *js_in)
{
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
	char *jstr = json_dumps(js, JSON_INDENT(4));
	if(jstr) {
		syslog(LOG_NOTICE, "Sending ...");
		send2(json_string_value(json_object_get(js_in, "src_host")), json_integer_value(json_object_get(js_in, "src_port")), jstr);
		free(jstr);
	} else {
		syslog(LOG_ERR, "Can not decode JSON");
	}
	json_decref(js);
}

void UServer::send2(const char *host, int port, const char *msg)
{
	struct sockaddr_in si_other;
	int s, slen=sizeof(si_other);
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		syslog(LOG_ERR, "Can not create UDP socket");
	} else {
		memset((char *) &si_other, 0, sizeof(si_other));
		si_other.sin_family = AF_INET;
		si_other.sin_port = htons(port);
		if(inet_aton(host, &si_other.sin_addr)==0) {
			syslog(LOG_ERR, "inet_aton() for %s failed", host);
		} else {
			syslog(LOG_NOTICE, "Sending %d bytes to %s:%d ...", strlen(msg), host, port);
			if(sendto(s, msg, strlen(msg), 0, (const sockaddr*)&si_other, slen)==-1) {
				syslog(LOG_ERR, "sendto() failed");
			}
			syslog(LOG_NOTICE, "%d bytes to %s:%d has been sent", strlen(msg), host, port);
		}
	}
	close(s);
}

