#include "userver.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <time.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

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

UServer::UServer(int port):ReServant("userver"),port(port),rvals_count(0)
{
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

void UServer::send_file(struct evhttp_request *req, const char *fname, const char *mime)
{
	char index_path[400];
	strcpy(index_path, mypath());
	strcat(index_path, "/");
	strcat(index_path, fname);
	struct evbuffer *buf = evbuffer_new();
	struct evkeyvalq *headers = evhttp_request_get_output_headers(req);
	int fd = open(index_path, O_RDONLY);
	if(fd >= 0) {
		struct stat st;
		fstat(fd, &st);
		evbuffer_add_file(buf, fd, 0, st.st_size);
		syslog(LOG_NOTICE, "file size=%lu, bufsize=%d", st.st_size, evbuffer_get_length(buf));
		evhttp_add_header(headers, "Content-Type", mime);
		evhttp_send_reply(req, 200, "OK", buf);
		close(fd);
	} else {
		const char *r404 = "404 Not found";
		evbuffer_add(buf, r404, strlen(r404));
		evhttp_send_reply(req, 404, "not found", buf);
	}
}

void UServer::http_request(struct evhttp_request *req)
{
	const char *cmdtype;
	struct evkeyvalq *headers;
	struct evkeyval *header;

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

	syslog(LOG_NOTICE, "Received a %s request for %s", cmdtype, evhttp_request_get_uri(req));

	headers = evhttp_request_get_input_headers(req);
	for (header = headers->tqh_first; header;
	    header = header->next.tqe_next) {
		syslog(LOG_NOTICE, "  %s: %s\n", header->key, header->value);
	}

	struct evbuffer *ibuf = evhttp_request_get_input_buffer(req);
	syslog(LOG_NOTICE, "Input data: <<<");
	while (evbuffer_get_length(ibuf)) {
		int n;
		char cbuf[128];
		n = evbuffer_remove(ibuf, cbuf, sizeof(cbuf)-1);
		if (n > 0)
			syslog(LOG_NOTICE, "%s", cbuf);
	}
	syslog(LOG_NOTICE, ">>>");

	if(strcmp(req->uri, "/") == 0) {
		send_file(req, "index.html");
	} else if(strcmp(req->uri, "/car_back") == 0) {
		send_file(req, "car_back.png", "image/png");
	} else if(strcmp(req->uri, "/car_side") == 0) {
		send_file(req, "car_side.png", "image/png");
	} else if(strcmp(req->uri, "/car_top") == 0) {
		send_file(req, "car_top.png", "image/png");
	} else {
		struct evbuffer *buf = evbuffer_new();
		headers = evhttp_request_get_output_headers(req);
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
			syslog(LOG_NOTICE, "%s\n", jstr);
			evbuffer_add(buf, jstr, strlen(jstr));
			evhttp_add_header(headers, "Content-Type", "application/json");
			evhttp_send_reply(req, 200, "OK", buf);
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not decode JSON\n");
			evhttp_send_reply(req, 500, "Internal Error", buf);
		}
		json_decref(js);
		evbuffer_free(buf);
	}
	syslog(LOG_NOTICE, "URI:%s", req->uri);

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

