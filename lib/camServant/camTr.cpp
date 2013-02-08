#include <reservant.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <string>

/*
#include <malloc.h>
#include <syslog.h>


#include <vector>
#include <iostream>

#include <hiredis.h>
#include <async.h>
#include <adapters/libevent.h>

#include <jansson.h>
*/
#include <CMUcam4.h>
#include <CMUcom4.h>

#define RED_MIN 20
#define RED_MAX 255
#define GREEN_MIN 23
#define GREEN_MAX 255
#define BLUE_MIN 23
#define BLUE_MAX 255

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds

const int servo_min = 750;
const int servo_max = 2250;
const int servo_steps = servo_max - servo_min;

#define E(func) (_E((func), __LINE__))

const char *s_timestamp()
{
	static char rs[50];
	time_t t = time(NULL);
	struct tm *st = localtime(&t);
	sprintf(rs, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec);
	return rs;
}

struct IMBUF {
	int isize;
	uint8_t ibuf[160*120*4];
};

class camTr:public ReServant
{
	private:
		long _E(long func, unsigned long line);
		void printH(int tilt, int p, const uint8_t *bins, int length);
		void cam_begin();
		void cam_end();
		void setup();
		void clearSD();
		int getImage(IMBUF &ibuf);
		const char *chan_color(int chan);
		void storeCurPos();
		void setCurPos();
		void restoreCurPos();
		void move_up(json_t *js);
		void move_down(json_t *js);
		void move_left(json_t *js);
		void move_right(json_t *js);
		void set_servo_hpos(json_t *js);
		void set_servo_vpos(json_t *js);
		void make_image(json_t *js);
		void set_histogram_mode(json_t *js);
		void set_window(json_t *js);

	protected:
		CMUcam4 cam;

		int h_chan;
		int h_bins;
		virtual void create_servant();
		virtual void loop();

		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
	public:
		typedef void (camTr::*tFunction)(json_t *js);

		camTr();
		void setCurPosCb(redisAsyncContext *c, void *r, int servo);
};

struct CAM_CMD_FUNC:public CMD_FUNC {
	public:
		CAM_CMD_FUNC(const char *cmd, camTr::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		camTr::tFunction ptr;
};

typedef CAM_CMD_FUNC *pCAM_CMD_FUNC;

camTr::camTr():ReServant("camTr"),h_chan(CMUCAM4_GREEN_CHANNEL),h_bins(CMUCAM4_H4_BINS),cam(CMUCOM4_SERIAL)
{
	const static pCAM_CMD_FUNC cmdlist[] = {
		new CAM_CMD_FUNC("move_up", &camTr::move_up),
		new CAM_CMD_FUNC("move_down", &camTr::move_down),
		new CAM_CMD_FUNC("move_left", &camTr::move_left),
		new CAM_CMD_FUNC("move_right", &camTr::move_right),
		new CAM_CMD_FUNC("set_servo_hpos", &camTr::set_servo_hpos),
		new CAM_CMD_FUNC("set_servo_vpos", &camTr::set_servo_vpos),
		new CAM_CMD_FUNC("make_image", &camTr::make_image),
		new CAM_CMD_FUNC("set_histogram_mode", &camTr::set_histogram_mode),
		new CAM_CMD_FUNC("set_window", &camTr::set_window)
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void camTr::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pCAM_CMD_FUNC ccmd = (pCAM_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
//	(this->&((void (camTr::*))(json_t*)))cmd->ptr(js);
}

long camTr::_E(long func, unsigned long line)
{
  if(func < CMUCAM4_RETURN_SUCCESS)
  {
	char s_err[256];
	sprintf(s_err, "Caught error %d on line %d", func, line);
	syslog(LOG_ERR, "\n%s\n", s_err);
    redisAsyncCommand(aredis, NULL, NULL, "SET %s.s.error.timestamp %s", myid(), s_timestamp());
    redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.error_code %d", myid(), func);
    redisAsyncCommand(aredis, NULL, NULL, "SET %s.s.cam_error %s", myid(), s_err);
  }

  return func;
}

void camTr::printH(int tilt, int p, const uint8_t *bins, int length)
{
//	redisAsyncCommand(aredis, NULL, NULL, "SET bin:%d:%d %b", tilt, p, bins, length*sizeof(uint8_t));
	char *buf = (char *)calloc(sizeof(char), length * 3 + 50);
	sprintf(buf, "%s,%d,%d", s_timestamp(), tilt, p);
	for(int i = 0; i< length; i++) {
		sprintf(buf + strlen(buf), ",%d", bins[i]);
	}
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.s.h%d:%d %s", myid(), tilt, p, buf, strlen(buf));
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


void camTr::cam_begin()
{
//	pinMode(17, OUTPUT);
//	pinMode(24, OUTPUT);
//	digitalWrite(17, false);
//	digitalWrite(24, true);
	cam.begin();
}

void camTr::cam_end()
{
	cam.end();
}

void camTr::setup()
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
//	cam_end();

//	cam.lineMode(true);
//	cam.automaticPan(1, 0);
//	cam.testMode(1);
	//syslog(LOG_DEBUG, "colorT=%d\n", cam.colorTracking(1));
	
}


void camTr::clearSD()
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

int camTr::getImage(IMBUF &ibuf)
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

const char *camTr::chan_color(int chan)
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

void camTr::storeCurPos()
{
    redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.tilt %d", myid(), cam.getServoPosition(CMUCAM4_TILT_SERVO));
    redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.pan %d", myid(), cam.getServoPosition(CMUCAM4_PAN_SERVO));
}

struct SPCb {
	camTr *ths;
	int servo;
	SPCb(camTr *ths, int servo) {
		this->ths = ths;
		this->servo = servo;
	}
};

void setCurPosCallback(redisAsyncContext *c, void *r, void *privdata)
{
	SPCb *pdata=(SPCb*)privdata;
	pdata->ths->setCurPosCb(c, r, pdata->servo);
	delete pdata; 
}

void camTr::setCurPosCb(redisAsyncContext *c, void *r, int servo) 
{
    redisReply *reply = (redisReply *)r;
    if (reply != NULL) {
		if(reply->str) {
			int pos = atoi(reply->str);
			syslog(LOG_NOTICE, "setting servo %d to %d\n", servo, pos);
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

void camTr::setCurPos()
{
    redisAsyncCommand(aredis, setCurPosCallback, new SPCb(this, CMUCAM4_TILT_SERVO), "GET set_tilt");
    redisAsyncCommand(aredis, setCurPosCallback, new SPCb(this, CMUCAM4_PAN_SERVO), "GET set_pan");
}

void camTr::restoreCurPos()
{
	char buf[50];
	sprintf(buf, "%d", CMUCAM4_TILT_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET %s.i.tilt", myid());
	sprintf(buf, "%d", CMUCAM4_PAN_SERVO);
    redisAsyncCommand(aredis, setCurPosCallback, strdup(buf), "GET %s.i.pan", myid());
}

void camTr::loop()
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
				redisAsyncCommand(aredis, NULL, NULL, "SET %s.s.%s_hist %s", myid(), chan_color(h_chan), buf, strlen(buf));
				free(buf);
			}
		}
	}
	struct timeval tv;
	gettimeofday(&tv, NULL);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.s.timestamp %d.%6.6d", myid(), tv.tv_sec, tv.tv_usec);
}


void camTr::move_up(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move up by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, cam.getServoPosition(CMUCAM4_TILT_SERVO) + value));
	storeCurPos();
}

void camTr::move_down(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move down by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, cam.getServoPosition(CMUCAM4_TILT_SERVO) - value));
	storeCurPos();
}

void camTr::move_right(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move right by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, cam.getServoPosition(CMUCAM4_PAN_SERVO) - value));
	storeCurPos();
}

void camTr::move_left(json_t *js)
{
	int value = json_integer_value(json_object_get(js, "value"));
	syslog(LOG_DEBUG, "move left by %d\n", value);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, cam.getServoPosition(CMUCAM4_PAN_SERVO) + value));
	storeCurPos();
}

void camTr::set_servo_hpos(json_t *js)
{
	int angle = json_integer_value(json_object_get(js, "angle"));
	int pos = (servo_max - servo_min)/ 180. * (angle + 90) + servo_min;
	syslog(LOG_DEBUG, "pan to %d\n", pos);
	E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, pos));
	storeCurPos();
}

void camTr::set_servo_vpos(json_t *js)
{
	int angle = json_integer_value(json_object_get(js, "angle"));
	int pos = (servo_max - servo_min)/ 180. * (angle + 90) + servo_min;
	syslog(LOG_DEBUG, "tilt to %d\n", pos);
	E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, pos));
	storeCurPos();
}

void camTr::make_image(json_t *js)
{
	IMBUF ibuf;
	if(getImage(ibuf) > 0) {
		redisAsyncCommand(aredis, NULL, NULL, "SET %s.b.image %b", myid(), ibuf.ibuf, ibuf.isize);
	}
}

void camTr::set_histogram_mode(json_t *js)
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

void camTr::set_window(json_t *js)
{
	CMUcam4_tracking_window_t tw;
	int topLeftX = json_integer_value(json_object_get(js, "x1"));
	int topLeftY = json_integer_value(json_object_get(js, "y1"));
	int bottomRightX = json_integer_value(json_object_get(js, "x2"));
	int bottomRightY = json_integer_value(json_object_get(js, "y2"));
	E(cam.setTrackingWindow(topLeftX, topLeftY, bottomRightX, bottomRightY));
	E(cam.getTrackingWindow(&tw));
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.l.cur_window %s,%d,%d,%d,%d", myid(), s_timestamp(), tw.topLeftX, tw.topLeftY, tw.bottomRightX, tw.bottomRightY);
}

void camTr::create_servant()
{
	ReServant::create_servant();
	setup();
}


int main (int argc, char **argv) 
{
	camTr srv = camTr();
	srv.run();
	return 0;
}
