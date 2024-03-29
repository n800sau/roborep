#ifndef __USERVER_H

#define __USERVER_H

#include <string.h>
#include <reservant.h>

static const char *rgroups[] = {
	"adxl345",
	"l3g4200d",
	"hmc5883l",
	"cmucam4",
	"kalman",
	"lsm303",
	"mag3110",
	"bmp085",
	"mpu6050",
};

static const int rgroups_count = sizeof(rgroups) / sizeof(*rgroups);

enum RTYPES {R_INT, R_REAL, R_STR, R_JSON};
static const char *rtypes[] = {"i", "r", "s", "js"};

struct RVAL {
	const char *key;
	RTYPES rtype;
	union {
		const char *sval;
		double rval;
		int ival;
	} u;
	inline RVAL(const char *key, const char *sval, RTYPES rtype=R_STR) { this->key = strdup(key); u.sval = strdup(sval); this->rtype = rtype; }
	inline RVAL(const char *key, double rval) { this->key = strdup(key); u.rval = rval; rtype = R_REAL; }
	inline RVAL(const char *key, int ival) { this->key = strdup(key); u.ival = ival; rtype = R_INT; }
	~RVAL() { free((void*) key); if (rtype == R_STR) free((void*) u.sval); }
};

#define MAX_RVALS_COUNT 100

class UServer:public ReServant
{
	private:
		int port;
		RVAL *rvals[MAX_RVALS_COUNT];
		int rvals_count;
		void send_full_data(json_t *js);
		void send2(const char *host, int port, const char *msg);
	protected:
		virtual void loop();
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		UServer(int port=0);
		~UServer();
		virtual bool create_servant();

		virtual void udp_request(sockaddr_in stFromAddr, const char *aReqBuffer);

		void keyCallback(redisAsyncContext *c, const char *rgroup, const char *rtype, redisReply *reply);
		void paramCallback(redisAsyncContext *c, const char *rkey, const char *rgroup, const char *rtype, redisReply *reply);
		typedef void (UServer::*tFunction)(json_t *js);

		void send_full_cb_func(json_t *js_in);

};

struct userver_redisReply {
	const char *rgroup;
	const char *rkey;
	const char *rtype;
	UServer *ths;
	userver_redisReply(UServer *ths, const char *rgroup, const char *rtype, const char *rkey=NULL);
	~userver_redisReply();
};

#endif //__USERVER_H
