#ifndef __RHTTP_H

#define __RHTTP_H

#include <reservant.h>

static const char *rgroups[] = {
	"adxl345",
	"l3g4200d",
	"hmc5883l",
	"cmucam4"
};

static const int rgroups_count = sizeof(rgroups) / sizeof(*rgroups);

#define MAXVALCOUNT 100
#define MAXKEYSIZE 20

class Rhttp:public ReServant
{
	private:
		int port;
		char rkeys[MAXVALCOUNT][MAXKEYSIZE];
		float rvals[MAXVALCOUNT];
		int rvals_count;
	protected:
		virtual void create_servant();
		virtual void loop();
	public:
		Rhttp(int port=0);

		virtual void http_request(struct evhttp_request *req);

		void paramCallback(redisAsyncContext *c, const char *rkey, const char rtype, redisReply *reply);
		void keyCallback(redisAsyncContext *c, const char rtype, redisReply *reply);
};

struct rhttp_redisReply {
	const char *rkey;
	char rtype;
	Rhttp *ths;
	inline rhttp_redisReply(Rhttp *ths, const char rtype, const char *rkey=NULL) { this->rkey = rkey; this->rtype = rtype; this->ths = ths; }
};

#endif //__RHTTP_H
