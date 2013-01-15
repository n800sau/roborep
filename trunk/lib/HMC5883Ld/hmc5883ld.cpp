#include <stdio.h>
#include <math.h>
#include <syslog.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>

#include <hiredis.h>
#include <async.h>
#include <adapters/libevent.h>

#include <jansson.h>

#include "HMC5883L.h"


const double PI  =3.141592653589793238462;

const char r_hmc5883l_cmd[] = "hmc5883l_cmd";

typedef void AFUNC(json_t *js);

struct CMD_FUNC {
	const char *cmd;
	AFUNC *func;
} cmdlist[] = {
};

int exiting = 0;
static int n_calls = 0;

struct event_base *base;
struct event *timer_ev;

redisAsyncContext *aredis;
redisContext *redis;

HMC5883L *compass;

// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   syslog(LOG_NOTICE, "Raw:\t%d\t%d\t%d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET raw.X %d", raw.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET raw.Y %d", raw.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET raw.Z %d", raw.ZAxis);

   syslog(LOG_NOTICE, "Scaled:\t%g\t%g\t%g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET scaled.X %g", scaled.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET scaled.Y %g", scaled.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET scaled.Z %g", scaled.ZAxis);

   syslog(LOG_NOTICE, "Heading:\t%g radians,\t%g degrees\n", heading, headingDegrees);
	redisAsyncCommand(aredis, NULL, NULL, "SET heading.radians %g", heading);
	redisAsyncCommand(aredis, NULL, NULL, "SET heading.degrees %g", headingDegrees);
}

void cmdCallback(redisAsyncContext *c, void *r, void *privdata) {
    redisReply *reply = (redisReply *)r;
    if (reply == NULL) {
		syslog(LOG_WARNING, "no reply\n");
	} else {
//		printf("lpop %d %p %d %d\n", reply->type, reply->str, REDIS_REPLY_ARRAY, REDIS_REPLY_STRING, REDIS_REPLY_NIL);
		if(reply->str) {
			json_error_t error;
			json_t *js = json_loads(reply->str, JSON_DECODE_ANY, &error);
			if (js == NULL) {
				syslog(LOG_ERR, "Error JSON decoding:%s", error.text);
			} else {
				json_t *cmd = json_object_get(js, "cmd");
				for(int i=0; i< sizeof(cmdlist) / sizeof(*cmdlist); i++) {
					CMD_FUNC *cf = &cmdlist[i];
					if(strcmp(cf->cmd, json_string_value(cmd)) == 0) {
						cf->func(js);
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
	redisAsyncCommand(aredis, cmdCallback, NULL, "LPOP %s", r_hmc5883l_cmd);

    /* Disconnect after receiving the reply to GET */
//    redisAsyncDisconnect(c);
}
void connectCallback(const redisAsyncContext *c) {
    ((void)c);
    syslog(LOG_NOTICE, "connected...\n");
}

void disconnectCallback(const redisAsyncContext *c, int status) {
    if (status != REDIS_OK) {
        syslog(LOG_ERR, c->errstr);
    }
    syslog(LOG_NOTICE, "disconnected...\n");
}

void loop()
{
  syslog(LOG_NOTICE, "Setting scale to +/- 1.3 Ga\n");
  int error = compass->SetScale(GAUSS_1_3); // Set the scale of the compass->
  if(error != 0) // If there is an error, print it out.
    syslog(LOG_ERR, "%d:%s\n", error, compass->GetErrorText(error));
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass->ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass->ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	//12 + 34/60E = 12.56667 / 180 * pi() = 0.2193297
  float declinationAngle = 0.2193297;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);
	struct timeval tv;
	gettimeofday(&tv, NULL);
	redisAsyncCommand(aredis, NULL, NULL, "SET timestamp %d.%6.6d", tv.tv_sec, tv.tv_usec);
}

void cb_func(evutil_socket_t fd, short what, void *arg)
{
    syslog(LOG_NOTICE, "cb_func called %d times so far.\n", ++n_calls);
	loop();
//    if (n_calls > 100)
//       event_del(timer_ev);
}


int main()
{
	setlogmask (LOG_UPTO (LOG_DEBUG));
	openlog("hmc5883l", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);

	syslog(LOG_NOTICE, "Hello from hmc5883ld\n");
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
        return 1;
    }
	compass = new HMC5883L(); // Construct a new HMC5883 compass

    redisLibeventAttach(aredis,base);
    redisAsyncSetConnectCallback(aredis,connectCallback);
    redisAsyncSetDisconnectCallback(aredis,disconnectCallback);
    redisAsyncCommand(aredis, cmdCallback, NULL, "LPOP %s", r_hmc5883l_cmd);
	timer_ev = event_new(base, -1, EV_PERSIST, cb_func, NULL);
	struct timeval one_sec = { 5, 0 };
	event_add(timer_ev, &one_sec);
	do {
    	event_base_loop(base, EVLOOP_NONBLOCK);
    } while(!exiting);
    event_base_dispatch(base);
	closelog();
    return 0;
}






