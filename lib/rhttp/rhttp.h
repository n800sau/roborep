#ifndef __RHTTP_H

#define __RHTTP_H

#include <string.h>
#include <reservant.h>

static const char *rgroups[] = {
	"adxl345",
	"l3g4200d",
	"hmc5883l",
	"cmucam4"
};

static const int rgroups_count = sizeof(rgroups) / sizeof(*rgroups);

enum RTYPES {R_INT, R_REAL, R_STR};
static const char *rtypes[] = {"i", "r", "s"};

struct RVAL {
	const char *key;
	RTYPES rtype;
	union {
		const char *sval;
		double rval;
		int ival;
	} u;
	inline RVAL(const char *key, const char *sval) { this->key = strdup(key); u.sval = strdup(sval); rtype = R_STR; }
	inline RVAL(const char *key, double rval) { this->key = strdup(key); u.rval = rval; rtype = R_REAL; }
	inline RVAL(const char *key, int ival) { this->key = strdup(key); u.ival = ival; rtype = R_INT; }
	~RVAL() { free((void*) key); if (rtype == R_STR) free((void*) u.sval); }
};

#define MAX_RVALS_COUNT 100

class Rhttp:public ReServant
{
	private:
		int port;
		RVAL *rvals[MAX_RVALS_COUNT];
		int rvals_count;
	protected:
		virtual void create_servant();
		virtual void loop();
	public:
		Rhttp(int port=0);
		~Rhttp();

		virtual void http_request(struct evhttp_request *req);
		void send_file(struct evhttp_request *req,  const char *fname, const char *mime="text/html");

		void keyCallback(redisAsyncContext *c, const char *rgroup, const char *rtype, redisReply *reply);
		void paramCallback(redisAsyncContext *c, const char *rkey, const char *rgroup, const char *rtype, redisReply *reply);
};

struct rhttp_redisReply {
	const char *rgroup;
	const char *rkey;
	const char *rtype;
	Rhttp *ths;
	rhttp_redisReply(Rhttp *ths, const char *rgroup, const char *rtype, const char *rkey=NULL);
	~rhttp_redisReply();
};

#endif //__RHTTP_H
