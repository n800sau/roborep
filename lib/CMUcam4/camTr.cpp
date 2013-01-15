#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <malloc.h>
#include <syslog.h>


#include <vector>
#include <iostream>
#include <string>

#include <hiredis.h>
#include <async.h>
#include <adapters/libevent.h>

#include <jansson.h>

#include "CMUcam4.h"
#include "CMUcom4.h"

const char r_cam_cmd[] = "cam_cmd";

#define RED_MIN 20
#define RED_MAX 255
#define GREEN_MIN 23
#define GREEN_MAX 255
#define BLUE_MIN 23
#define BLUE_MAX 255

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);
struct event_base *base;
redisAsyncContext *aredis;
redisContext *redis;
int exiting = 0;
int tilt_ndx = 0;
int pan_ndx = 0;
int pan_dir = 1;
const int servo_min = 750;
const int servo_max = 2250;
const int servo_steps = servo_max - servo_min;


int h_chan = CMUCAM4_GREEN_CHANNEL;
int h_bins = CMUCAM4_H4_BINS;

#define E(func) (_E((func), __LINE__))

const char *s_timestamp()
{
	static char rs[50];
	time_t t = time(NULL);
	struct tm *st = localtime(&t);
	sprintf(rs, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec);
	return rs;
}

long _E(long func, unsigned long line)
{
  if(func < CMUCAM4_RETURN_SUCCESS)
  {
	char s_err[256];
	sprintf(s_err, "Caught error %d on line %d", func, line);
	syslog(LOG_ERR, "\n%s\n", s_err);
    redisAsyncCommand(aredis, NULL, NULL, "SET cam_error_code %s,%d", s_timestamp(), func);
    redisAsyncCommand(aredis, NULL, NULL, "SET cam_error %s,%s", s_timestamp(), s_err);
  }

  return func;
}

void printH(int tilt, int p, const uint8_t *bins, int length)
{
//	redisAsyncCommand(aredis, NULL, NULL, "SET bin:%d:%d %b", tilt, p, bins, length*sizeof(uint8_t));
	char *buf = (char *)calloc(sizeof(char), length * 3 + 50);
	sprintf(buf, "%s,%d,%d", s_timestamp(), tilt, p);
	for(int i = 0; i< length; i++) {
		sprintf(buf + strlen(buf), ",%d", bins[i]);
	}
	redisAsyncCommand(aredis, NULL, NULL, "SET h%d:%d %s", tilt, p, buf, strlen(buf));
	FILE *f = fopen("bins.txt", "a");
	if(f) {
		fputs(buf, f);
		fputs("\n", f);
		fclose(f);
	} else {
		syslog(LOG_ERR, "Can not open log file\n");
		exit(1);
	}
	free(buf);
}


void cam_begin()
{
//	pinMode(17, OUTPUT);
//	pinMode(24, OUTPUT);
//	digitalWrite(17, false);
//	digitalWrite(24, true);
	cam.begin();
}

void cam_end()
{
	cam.end();
}

void setup()
{
  cam_begin();

  // Wait for auto gain and auto white balance to run.

  cam.LEDOn(LED_BLINK);
  usleep(1000*WAIT_TIME);

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);

  cam.LEDOn(CMUCAM4_LED_ON);
  cam.formatDisk();
	cam.monitorOn();
	//cam.automaticPan(1, 0);
	cam_end();

//	cam.lineMode(true);
//	cam.automaticPan(1, 0);
//	cam.testMode(1);
	//syslog(LOG_DEBUG, "colorT=%d\n", cam.colorTracking(1));
	tilt_ndx = 0;
	pan_ndx = 0;
	
}


void clearSD()
{
#define DE_SIZE 4
	CMUcam4_directory_entry_t de[DE_SIZE]; // Directory entry array.
	int directorySize;
	do {
		directorySize = E(cam.listDirectory(de, DE_SIZE, 0));
		if(directorySize > 0) {
			for(int i=0; i < std::min(DE_SIZE, directorySize); i++) {
				E(cam.removeEntry(de[i].name));
			}
		}
	} while(directorySize > DE_SIZE);
}

struct IMBUF {
	int isize;
	uint8_t ibuf[160*120*4];
};

int getImage(IMBUF &ibuf)
{
#define DE_SIZE 4
	ibuf.isize = 0;
	CMUcam4_directory_entry_t de[DE_SIZE]; // Directory entry array.
	if(E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120)) == 0) {
		int directorySize = E(cam.listDirectory(de, DE_SIZE, 0));
		if(directorySize > 0) {
			const char *fname = de[directorySize - 1].name;
			syslog(LOG_NOTICE, "File name:%s\n", fname);
			ibuf.isize = E(cam.filePrint(fname, ibuf.ibuf, sizeof(ibuf.ibuf), 0));
			if(ibuf.isize < 0) {
				syslog(LOG_ERR, "Error reading file %s\n", fname);
			}
		}
	}
	return ibuf.isize;
}

const char *chan_color(int chan)
{
	const char *rs = "none";
	switch(chan) {
		case CMUCAM4_GREEN_CHANNEL:
			rs = "green";
			break;
		case CMUCAM4_RED_CHANNEL:
			rs = "red";
			break;
		case CMUCAM4_BLUE_CHANNEL:
			rs = "blue";
			break;
	}
	return rs;
}

void storeCurPos()
{
    redisAsyncCommand(aredis, NULL, NULL, "SET cur_tilt %d", cam.getServoPosition(CMUCAM4_TILT_SERVO));
    redisAsyncCommand(aredis, NULL, NULL, "SET cur_pan %d", cam.getServoPosition(CMUCAM4_PAN_SERVO));
}

void setCurPosCallback(redisAsyncContext *c, void *r, void *privdata) {
    redisReply *reply = (redisReply *)r;
    if (reply != NULL) {
		if(reply->str) {
			int servo = atoi((char*)privdata);
			int pos = atoi(reply->str);
			syslog(LOG_NOTICE, "setting servo %s to %d\n", (char*)privdata, pos);
			free(privdata);
			E(cam.setServoPosition(servo, true, pos));
			switch(servo) {
				case CMUCAM4_PAN_SERVO:
					redisAsyncCommand(aredis, NULL, NULL, "DEL set_pan");
					break;
				case CMUCAM4_TILT_SERVO:
					redisAsyncCommand(aredis, NULL, NULL, "DEL set_tilt");
					break;
			}
			storeCurPos();
		}
	}
}

void setCurPos()
{
	char buf[50];
	sprintf(buf, "%d", CMUCAM4_TILT_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET set_tilt");
	sprintf(buf, "%d", CMUCAM4_PAN_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET set_pan");
}

void restoreCurPos()
{
	char buf[50];
	sprintf(buf, "%d", CMUCAM4_TILT_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET cur_tilt");
	sprintf(buf, "%d", CMUCAM4_PAN_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET cur_pan");
}

void loop()
{
	CMUcam4_image_data_t f_data;
	CMUcam4_tracking_data_t t_data;
	CMUcam4_histogram_data_1_t h_1_data;
	CMUcam4_histogram_data_2_t h_2_data;
	CMUcam4_histogram_data_4_t h_4_data;
	CMUcam4_histogram_data_8_t h_8_data;
	CMUcam4_histogram_data_16_t h_16_data;
	CMUcam4_histogram_data_32_t h_32_data;
	CMUcam4_histogram_data_64_t h_64_data;
	int length;
	uint8_t *bins;
	setCurPos();
	int rs = E(cam.getHistogram(h_chan, h_bins));
	if ( rs == 0 ) {
		switch(h_bins) {
			case CMUCAM4_H1_BINS:
				rs = E(cam.getTypeHDataPacket(&h_1_data)); // Get a histogram packet.
				length = CMUCAM4_HD_1_T_LENGTH;
				bins = h_1_data.bins;
				break;
			case CMUCAM4_H2_BINS:
				rs = E(cam.getTypeHDataPacket(&h_2_data)); // Get a histogram packet.
				length = CMUCAM4_HD_2_T_LENGTH;
				bins = h_2_data.bins;
				break;
			case CMUCAM4_H4_BINS:
				rs = E(cam.getTypeHDataPacket(&h_4_data)); // Get a histogram packet.
				length = CMUCAM4_HD_4_T_LENGTH;
				bins = h_4_data.bins;
				break;
			case CMUCAM4_H8_BINS:
				rs = E(cam.getTypeHDataPacket(&h_8_data)); // Get a histogram packet.
				length = CMUCAM4_HD_8_T_LENGTH;
				bins = h_8_data.bins;
				break;
			case CMUCAM4_H16_BINS:
				rs = E(cam.getTypeHDataPacket(&h_16_data)); // Get a histogram packet.
				length = CMUCAM4_HD_16_T_LENGTH;
				bins = h_16_data.bins;
				break;
			case CMUCAM4_H32_BINS:
				rs = E(cam.getTypeHDataPacket(&h_32_data)); // Get a histogram packet.
				length = CMUCAM4_HD_32_T_LENGTH;
				bins = h_32_data.bins;
				break;
			case CMUCAM4_H64_BINS:
				rs = E(cam.getTypeHDataPacket(&h_64_data)); // Get a histogram packet.
				length = CMUCAM4_HD_64_T_LENGTH;
				bins = h_64_data.bins;
				break;
			default:
				length = 0;
				bins = 0;
				break;
		}
		if(length > 0) {
			char *buf = (char *)calloc(sizeof(char), length * 3 + 50);
			if(buf) {
				time_t t = time(NULL);
				struct tm *st = localtime(&t);
				sprintf(buf, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec);
				for(int i = 0; i< length; i++) {
					sprintf(buf + strlen(buf), ",%d", bins[i]);
				}
				redisAsyncCommand(aredis, NULL, NULL, "SET %s_hist %s", chan_color(h_chan), buf, strlen(buf));
				free(buf);
			}
		}
	}
	struct timeval tv;
	gettimeofday(&tv, NULL);
	redisAsyncCommand(aredis, NULL, NULL, "SET timestamp %d.%6.6d", tv.tv_sec, tv.tv_usec);
}


void loop1()
{
	CMUcam4_image_data_t f_data;
	CMUcam4_tracking_data_t t_data;

			usleep(1000000L);
			uint16_t buffer[80*80]; // 10 by 10 pixel buffer
			uint8_t buffer3[80*80][3]; // 10 by 10 pixel buffer
			memset(buffer, 0, sizeof(buffer));

//			cam.pollMode(true);
//			cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);
//			cam.getTypeTDataPacket(&t_data); // Get a tracking packet.
//			memset(&f_data, 0, sizeof(f_data));
//			cam.getTypeFDataPacket(&f_data); // Get an image packet.
//			printf("sendBitmap ret=%d\n", cam.sendBitmap(&f_data));
//			for(int y = 0; y < CMUCAM4_ID_T_R ; y++) {
//				for(int x = 0; x < CMUCAM4_ID_T_C ; x++) {
//					printf("%.2X ", f_data.pixels[y * CMUCAM4_ID_T_C + x]);
//				}
//				printf("\n");
//			}
//			printf("\n");
//			usleep(1000000L);
}

void move_up(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move up by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, cam.getServoPosition(CMUCAM4_TILT_SERVO) + value));
	storeCurPos();
}

void move_down(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move down by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, cam.getServoPosition(CMUCAM4_TILT_SERVO) - value));
	storeCurPos();
}

void move_right(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move right by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, cam.getServoPosition(CMUCAM4_PAN_SERVO) - value));
	storeCurPos();
}

void move_left(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move left by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, cam.getServoPosition(CMUCAM4_PAN_SERVO) + value));
	storeCurPos();
}

void set_servo_hpos(json_t *js)
{
	int angle = json_integer_value(json_object_get(js, "angle"));
	int pos = (servo_max - servo_min)/ 180. * (angle + 90) + servo_min;
	syslog(LOG_DEBUG, "pan to %d\n", pos);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, pos));
	storeCurPos();
}

void set_servo_vpos(json_t *js)
{
	int angle = json_integer_value(json_object_get(js, "angle"));
	int pos = (servo_max - servo_min)/ 180. * (angle + 90) + servo_min;
	syslog(LOG_DEBUG, "tilt to %d\n", pos);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, pos));
	storeCurPos();
}

int next_id()
{
	return ((redisReply*)redisCommand(redis, "HINCR counter"))->integer;
}

void make_image(json_t *js)
{
	IMBUF ibuf;
	if(getImage(ibuf) > 0) {
		redisAsyncCommand(aredis, NULL, NULL, "SET cur_image %b", ibuf.ibuf, ibuf.isize);
	}
}

void set_histogram_mode(json_t *js)
{
	int nbins = json_integer_value(json_object_get(js, "bins"));
	switch(nbins) {
		case 1:
			h_bins = CMUCAM4_H1_BINS;
			break;
		case 2:
			h_bins = CMUCAM4_H2_BINS;
			break;
		case 4:
			h_bins = CMUCAM4_H4_BINS;
			break;
		case 8:
			h_bins = CMUCAM4_H8_BINS;
			break;
		case 16:
			h_bins = CMUCAM4_H16_BINS;
			break;
		case 32:
			h_bins = CMUCAM4_H32_BINS;
			break;
		case 64:
			h_bins = CMUCAM4_H64_BINS;
			break;
	}
	const char *chan = json_string_value(json_object_get(js, "chan"));
	if(strcmp(chan, "green") == 0) {
		h_chan = CMUCAM4_GREEN_CHANNEL;
	} else if (strcmp(chan, "blue") == 0) {
		h_chan = CMUCAM4_BLUE_CHANNEL;
	} else if (strcmp(chan, "red") == 0) {
		h_chan = CMUCAM4_RED_CHANNEL;
	}
}

void set_window(json_t *js) {
	CMUcam4_tracking_window_t tw;
	int topLeftX = json_integer_value(json_object_get(js, "x1"));
	int topLeftY = json_integer_value(json_object_get(js, "y1"));
	int bottomRightX = json_integer_value(json_object_get(js, "x2"));
	int bottomRightY = json_integer_value(json_object_get(js, "y2"));
	E(cam.setTrackingWindow(topLeftX, topLeftY, bottomRightX, bottomRightY));
	E(cam.getTrackingWindow(&tw));
	redisAsyncCommand(aredis, NULL, NULL, "SET cur_window %s,%d,%d,%d,%d", s_timestamp(), tw.topLeftX, tw.topLeftY, tw.bottomRightX, tw.bottomRightY);
}

typedef void AFUNC(json_t *js);

struct CMD_FUNC {
	const char *cmd;
	AFUNC *func;
} cmdlist[] = {
	{ "move_up", move_up },
	{ "move_down", move_down },
	{ "move_left", move_left },
	{ "move_right", move_right },
	{ "set_servo_hpos", set_servo_hpos },
	{ "set_servo_vpos", set_servo_vpos },
	{ "make_image", make_image },
	{ "set_histogram_mode", set_histogram_mode },
	{ "set_window", set_window }
};

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
	redisAsyncCommand(aredis, cmdCallback, NULL, "LPOP %s", r_cam_cmd);

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

static int n_calls = 0;
struct event *timer_ev;

void cb_func(evutil_socket_t fd, short what, void *arg)
{
    syslog(LOG_NOTICE, "cb_func called %d times so far.\n", ++n_calls);
	loop();
//    if (n_calls > 100)
//       event_del(timer_ev);
}


int main (int argc, char **argv) {
	setlogmask (LOG_UPTO (LOG_DEBUG));
	openlog("camTr", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);

	syslog(LOG_NOTICE, "Hello from camTr\n");
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
	setup();
	cam_begin();
	restoreCurPos();
    redisLibeventAttach(aredis,base);
    redisAsyncSetConnectCallback(aredis,connectCallback);
    redisAsyncSetDisconnectCallback(aredis,disconnectCallback);
    redisAsyncCommand(aredis, cmdCallback, NULL, "LPOP %s", r_cam_cmd);
	timer_ev = event_new(base, -1, EV_PERSIST, cb_func, NULL);
	struct timeval one_sec = { 5, 0 };
	event_add(timer_ev, &one_sec);
	do {
    	event_base_loop(base, EVLOOP_NONBLOCK);
    } while(!exiting);
    event_base_dispatch(base);
	cam_end();
	closelog();
    return 0;
}

