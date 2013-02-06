#include "rhttp.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

const char *s_timestamp()
{
	static char rs[50];
	time_t t = time(NULL);
	struct tm *st = localtime(&t);
	sprintf(rs, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec);
	return rs;
}

void Rhttp_keyCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	rhttp_redisReply *rr = (rhttp_redisReply *)privdata;
	rr->ths->keyCallback(c, rr->rtype, (redisReply *)r);
	delete rr;
}

void Rhttp_paramCallBack(redisAsyncContext *c, void *r, void *privdata)
{
	rhttp_redisReply *rr = (rhttp_redisReply *)privdata;
	rr->ths->paramCallback(c, rr->rkey, rr->rtype, (redisReply *)r);
	delete rr;
}

Rhttp::Rhttp(int port):ReServant("rhttp"),port(port),rvals_count(0)
{
}

void Rhttp::create_servant()
{
	ReServant::create_servant();
//	redisAsyncCommand(aredis, Rhttp_keyCallBacke, NULL, "SUBSCRIBE ");
	runHttpd("0.0.0.0", port);
}

void Rhttp::loop()
{
	for(int i=0; i<rgroups_count; i++) {
		rhttp_redisReply *rr;
		rr = new rhttp_redisReply(this, 'r');
		redisAsyncCommand(aredis, Rhttp_keyCallBack, rr, "KEYS %s.%c.*", rgroups[i], rr->rtype);
		rr = new rhttp_redisReply(this, 'i');
		redisAsyncCommand(aredis, Rhttp_keyCallBack, rr, "KEYS %s.%c.*", rgroups[i], rr->rtype);
		rr = new rhttp_redisReply(this, 's');
		redisAsyncCommand(aredis, Rhttp_keyCallBack, rr, "KEYS %s.%c.*", rgroups[i], rr->rtype);
	}
	ReServant::loop();
}

int fsize(FILE *fp){
    int prev=ftell(fp);
    fseek(fp, 0L, SEEK_END);
    int sz=ftell(fp);
    fseek(fp,prev,SEEK_SET); //go back to where we were
    return sz;
}

void Rhttp::http_request(struct evhttp_request *req)
{
//	ReServant::http_request(req);
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

	syslog(LOG_NOTICE, "Received a %s request for %s", cmdtype, evhttp_request_get_uri(req));

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

	buf = evbuffer_new();
	if(strcmp(req->uri, "/") == 0) {
		char index_path[400];
		strcpy(index_path, mypath());
		strcat(index_path, "/index.html");
		int fd = open(index_path, O_RDONLY);
		if(fd >= 0) {
			struct stat st;
			fstat(fd, &st);
			evbuffer_add_file(buf, fd, 0, st.st_size);
//			evbuffer_read(buf, fd, -1);
			syslog(LOG_NOTICE, "file size=%d, bufsize=%d", st.st_size, evbuffer_get_length(buf));
			evhttp_send_reply(req, 200, "OK", buf);
			close(fd);
		} else {
			syslog(LOG_WARNING, "Error opening %s", index_path);
			evhttp_send_reply(req, 500, "Internal Error", buf);
		}
	} else {
		json_t *js = json_object();
		json_object_set_new(js, "mypath()", json_string(mypath()));
		json_object_set_new(js, "s_timestamp", json_string(s_timestamp()));
		for(int i=0; i<rvals_count; i++) {
			json_object_set_new(js, rkeys[i], json_real(rvals[i]));
		}
		char *jstr = json_dumps(js, JSON_INDENT(4));
		if(jstr) {
			syslog(LOG_DEBUG, "%s\n", jstr);
			evbuffer_add(buf, jstr, strlen(jstr));
			headers = evhttp_request_get_output_headers(req);
			evhttp_add_header(headers, "Content-Type", "application/json");
			evhttp_send_reply(req, 200, "OK", buf);
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not decode JSON\n");
			evhttp_send_reply(req, 500, "Internal Error", buf);
		}
		json_decref(js);
	}
	evbuffer_free(buf);
	syslog(LOG_NOTICE, "URI:%s", req->uri);

}

void Rhttp::keyCallback(redisAsyncContext *c, const char rtype, redisReply *reply)
{
	if (reply == NULL) {
		syslog(LOG_WARNING, "no keys in reply\n");
	} else {
		if(reply->type == REDIS_REPLY_ARRAY) {
			printf(">>>%s, %d %d %d\n", reply->str, reply->elements, reply->type, reply->len);
			for(int i=0;i<reply->elements; i++) {
				redisReply *el = reply->element[i];
				printf("key reply %s\n", el->str);
				rhttp_redisReply *rr = new rhttp_redisReply(this, rtype, el->str);
				redisAsyncCommand(aredis, Rhttp_paramCallBack, rr, "GET %s", el->str);
			}
		}
	}
}

void Rhttp::paramCallback(redisAsyncContext *c, const char *rkey, const char rtype, redisReply *reply)
{
	if (reply == NULL) {
		syslog(LOG_WARNING, "no reply\n");
	} else {
		if(reply->str) {
			printf("param reply %s(%c)=%s\n", rkey, rtype, reply->str);
		}
	}
}

