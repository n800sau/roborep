#include "kalman.h"
#include <math.h>
#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <hiredis_ext.h>

const double PI = 3.141592653589793238462;
const double RAD_TO_DEG = 57.29578;

struct KALMAN_CMD_FUNC:public CMD_FUNC {
	public:
		KALMAN_CMD_FUNC(const char *cmd, Kalman::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		Kalman::tFunction ptr;
};

typedef KALMAN_CMD_FUNC *pKALMAN_CMD_FUNC;

Kalman::Kalman():ReServant("kalman"),x(),y(),z()
{
	x.k.setAngle(270); // The angle calculated by accelerometer starts at 270 degrees
	const static pKALMAN_CMD_FUNC cmdlist[] = {
		new KALMAN_CMD_FUNC("stop_state", &Kalman::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void Kalman::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pKALMAN_CMD_FUNC ccmd = (pKALMAN_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void Kalman::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
	//requesting accelerator and gyro stop states
	redisReply *reply;
	reply = (redisReply *)redisCommand(redis, "INCRBY sequence 1");
	int adxl345_n = reply->integer;
	char buf[256];
	sprintf(buf, "{\"cmd\": \"stop_state\", \"reply_key\": \"reply_adxl345_%d\"}", adxl345_n);
	redisCommandN(redis, 3, "LPUSH", "cmd.adxl345", buf);
	reply = (redisReply *)redisCommand(redis, "INCRBY sequence 1");
	int l3g4200d_n = reply->integer;
	sprintf(buf, "{\"cmd\": \"stop_state\", \"reply_key\": \"reply_l3g4200d_%d\"}", l3g4200d_n);
	redisCommandN(redis, 3, "LPUSH", "cmd.l3g4200d", buf);

	syslog(LOG_DEBUG, "Waiting for adxl345 reply");
	reply = (redisReply *)redisCommand(redis, "BRPOP reply_adxl345_%d 50", adxl345_n);
	if(reply) {
		syslog(LOG_DEBUG, "adxl345 replied %s", reply->element[1]->str);
	} else {
		syslog(LOG_DEBUG, "adxl345 does not reply");
	}

	syslog(LOG_DEBUG, "Waiting for l3g4200d reply");
	reply = (redisReply *)redisCommand(redis, "BRPOP reply_l3g4200d_%d 50", l3g4200d_n);
	if(reply) {
		syslog(LOG_DEBUG, "l3g4200d replied %s", reply->element[1]->str);
	} else {
		syslog(LOG_DEBUG, "l3g4200d does not reply");
	}
}

bool Kalman::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		redisCommandN(redis, 3, "LPUSH", "cmd.kalman", "{\"cmd\": \"stop_state\"}");
	}
	return rs;
}

bool Kalman::fill_json(json_t *js, int list_id)
{
	json_t *sjs = json_object();
	json_object_set_new(sjs, "angle", json_real(x.angle));
	json_object_set_new(sjs, "rate", json_real(x.rate));
	json_object_set_new(sjs, "pitch", json_real(x.pitch));
	json_object_set_new(sjs, "timer", json_real(x.timer));
	json_object_set_new(js, "x", sjs);
	return true;
}

void Kalman::loop()
{
	redisReply *reply;
	json_error_t jserror;
	reply = (redisReply*)redisCommand(redis, "LINDEX %s.js.obj -1", "adxl345");
//	syslog(LOG_NOTICE, reply->str);
	json_t *js = json_loads(reply->str, JSON_DISABLE_EOF_CHECK, &jserror);
	freeReplyObject(reply);
	if(js) {
		x.angle = getAngleX(json_object_get(js, "scaled"), json_object_get(js, "stop_scaled"), "x", "z");
		reply = (redisReply*)redisCommand(redis, "LINDEX %s.js.obj -1", "l3g4200d");
//		syslog(LOG_NOTICE, reply->str);
		js = json_loads(reply->str, JSON_DISABLE_EOF_CHECK, &jserror);
		freeReplyObject(reply);
		if(js) {
			x.rate = getRateX(json_object_get(js, "curr_g"), json_object_get(js, "stop_g"), "x");
			x.pitch = x.k.getAngle(x.angle, x.rate, double(dtime() - x.timer)/1000000);  // Calculate the angle using the Kalman filter
			x.timer = dtime();
			json2redislist();
		} else {
			syslog(LOG_NOTICE, "Error while reading l3g4200d list: %s", jserror.text);
		}
	} else {
		syslog(LOG_NOTICE, "Error while reading adxl345 list: %s", jserror.text);
	}

	ReServant::loop();
}

double Kalman::getAngleX(json_t *acc_js, json_t *stop_acc_js, const char *X, const char *Z) {
  double accXval = json_real_value(json_object_get(acc_js, X)) - json_real_value(json_object_get(stop_acc_js, X));
  double accZval = (double)json_real_value(json_object_get(acc_js, Z)) - json_real_value(json_object_get(stop_acc_js, Z));
  double angle = (atan2(accXval,accZval)+PI)*RAD_TO_DEG;
  return angle;
}

double Kalman::getRateX(json_t *gyro_js, json_t *stop_gyro_js, const char *X) {
  return -((json_real_value(json_object_get(gyro_js, X)) - json_real_value(json_object_get(stop_gyro_js, X)))/14.375);
}
