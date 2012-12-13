#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#include <vector>
#include <iostream>
#include <string>

#include "hiredis.h"
#include "async.h"
#include "adapters/libevent.h"

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
redisAsyncContext *redis;
int exiting = 0;
int tilt_ndx = 0;
int pan_ndx = 0;
int pan_dir = 1;
const int servo_steps = 2250 - 750;


#define E(func) (_E((func), __LINE__))

long _E(long func, unsigned long line)
{
  if(func < CMUCAM4_RETURN_SUCCESS)
  {
    printf("\nCaught error %d on line %d\n", func, line);
  }

  return func;
}

void printH(int tilt, int p, const uint8_t *bins, int length)
{
//	redisAsyncCommand(redis, NULL, NULL, "SET bin:%d:%d %b", tilt, p, bins, length*sizeof(uint8_t));
	char *buf = (char *)calloc(sizeof(char), length * 3 + 50);
	time_t t = time(NULL);
	struct tm *st = localtime(&t);
	sprintf(buf, "%.4d.%.2d.%.2d %.2d:%.2d:%.2d,%d,%d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec, tilt, p);
	for(int i = 0; i< length; i++) {
		sprintf(buf + strlen(buf), ",%d", bins[i]);
	}
	redisAsyncCommand(redis, NULL, NULL, "SET h%d:%d %s", tilt, p, buf, strlen(buf));
	FILE *f = fopen("bins.txt", "a");
	if(f) {
		fputs(buf, f);
		fputs("\n", f);
		fclose(f);
	} else {
		printf("Can not open log file\n");
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
	//printf("colorT=%d\n", cam.colorTracking(1));
	tilt_ndx = 0;
	pan_ndx = 0;
	
}


void loop()
{
	CMUcam4_image_data_t f_data;
	CMUcam4_tracking_data_t t_data;

#define HISTOGRAM_CHANNEL CMUCAM4_GREEN_CHANNEL
#define HISTOGRAM_BINS CMUCAM4_H4_BINS
  CMUcam4_histogram_data_1_t h_1_data;
  CMUcam4_histogram_data_2_t h_2_data;
  CMUcam4_histogram_data_4_t h_4_data;
  CMUcam4_histogram_data_8_t h_8_data;
  CMUcam4_histogram_data_16_t h_16_data;
  CMUcam4_histogram_data_32_t h_32_data;
  CMUcam4_histogram_data_64_t h_64_data;

//	printf("\nt=%d, p=%d:\n", tilt_ndx, pan_ndx);
	cam.setServoPosition(CMUCAM4_TILT_SERVO, true, tilt_ndx+750);
	cam.setServoPosition(CMUCAM4_PAN_SERVO, true, pan_ndx+750);


#define DE_SIZE 4
		CMUcam4_directory_entry_t de[DE_SIZE]; // Directory entry array.
		int directorySize;
		do {
			directorySize = E(cam.listDirectory(de, DE_SIZE, 0));
			if(directorySize > 0) {
				for(int i=0; i < std::min(DE_SIZE, directorySize); i++) {
					cam.removeEntry(de[i].name);
				}
			}
		} while(directorySize > DE_SIZE);
		printf("Dir size=%d\n", directorySize);

			usleep(1000000L);
			printf("histogram=%d\n", cam.getHistogram(HISTOGRAM_CHANNEL, HISTOGRAM_BINS));
			uint16_t buffer[80*80]; // 10 by 10 pixel buffer
			uint8_t buffer3[80*80][3]; // 10 by 10 pixel buffer
			memset(buffer, 0, sizeof(buffer));
/*			printf("sendFrame=%d\n", cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_HR_80, buffer, 80, 0, 80, 0));

			char fname[256];
			sprintf(fname, "%d.%d.jpg", tilt_ndx, pan_ndx);
	
			for(int k=0; k<80*80; k++)
			{
				buffer3[k][0] = buffer[k] & 0x1f;
				buffer3[k][1] = (buffer[k] << 5) & 0x3f;
				buffer3[k][2] = (buffer[k] << 11) & 0x1f;
			}

			int channels = 3;
			IplImage* cv_image = cvCreateImageHeader(cvSize(80,80), IPL_DEPTH_8U, channels);
			if (!cv_image)
			{
				printf("Error creating image\n");    // print error, failed to allocate image!
			}

			printf("Width step:%d\n", cv_image->widthStep);
			cvSetData(cv_image, buffer3, cv_image->widthStep);

			int pan_ndx[3] = {CV_IMWRITE_JPEG_QUALITY, 10, 0};
			cvSaveImage(fname, cv_image, pan_ndx);
			printf("end\n");

			// release resources
			cvReleaseImageHeader(&cv_image);
*/

    #if(HISTOGRAM_BINS == CMUCAM4_H1_BINS)
      cam.getTypeHDataPacket(&h_1_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_1_data.bins, CMUCAM4_HD_1_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H2_BINS)
      cam.getTypeHDataPacket(&h_2_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_2_data.bins, CMUCAM4_HD_2_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H4_BINS)
      cam.getTypeHDataPacket(&h_4_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_4_data.bins, CMUCAM4_HD_4_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H8_BINS)
      cam.getTypeHDataPacket(&h_8_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_8_data.bins, CMUCAM4_HD_8_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H16_BINS)
      cam.getTypeHDataPacket(&h_16_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_16_data.bins, CMUCAM4_HD_16_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H32_BINS)
      cam.getTypeHDataPacket(&h_32_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_32_data.bins, CMUCAM4_HD_64_T_LENGTH);

    #elif((HISTOGRAM_BINS == CMUCAM4_H64_BINS) && \
    (HISTOGRAM_CHANNEL == CMUCAM4_GREEN_CHANNEL))
      cam.getTypeHDataPacket(&h_64_data); // Get a histogram packet.
		printH(tilt_ndx, pan_ndx, h_64_data.bins, CMUCAM4_HD_64_T_LENGTH);

    #else // 1 to 64 bins...
      #error "Invalid number of bins"

    #endif

	if(E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120)) == 0) {
		directorySize = E(cam.listDirectory(de, DE_SIZE, 0));
		if(directorySize > 0) {
			const char *fname = de[directorySize - 1].name;
			uint8_t filebuf[160*120*4];
			char dfname[100];
			printf("File name:%s\n", fname);
			long fsize = cam.filePrint(fname, filebuf, sizeof(filebuf), 0);
			if(fsize >= 0) {
				sprintf(dfname, "%d_%d.bmp", tilt_ndx, pan_ndx);
				FILE *df = fopen(dfname, "wb");
				if (df) {
					fwrite(filebuf, 1, fsize, df);
					fclose(df);
				} else {
					printf("Error writing file %s\n", dfname);
				}
			} else {
				printf("Error reading file %s\n", fname);
			}
		}
	}



/*			cv::Mat M(80,80, CV_16UC3, buffer);

			printf("col:%d,rows:%d\n", M.cols, M.rows);
			createAlphaMat(M);
			printf("here1\n");


			
			std::vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);

/*			printf("here2\n");
			for(int y = 0; y < 40 ; y++) {
				for(int x = 0; x < 40 ; x++) {
					printf("%4.4X", (int)M.data[x+y*80]);
				}
			}*/
/*			imwrite(fname, M, compression_params);
			printf("here3\n");
*/


//			cv::FileStorage fs(fname, cv::FileStorage::WRITE);
//			fs << M;
//			fs.release();
//			for(int y = 0; y < 40 ; y++) {
//				for(int x = 0; x < 40 ; x++) {
//					if(buffer[y * 80 + x]) {
//						printf("%4.4X", (int)buffer[y * 80 + x]);
//					}
//				}
				//printf("\n");
//			}
			//printf("\n");
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
	pan_ndx += 40 * pan_dir;
	if(pan_ndx >= servo_steps || pan_ndx < 0) {
		tilt_ndx += 40;
		pan_dir *= -1;
		pan_ndx += 40 * pan_dir;
		if(tilt_ndx >= servo_steps / 2) {
			tilt_ndx = 0;
			exit(0);
		}
	}
}



void createAlphaMat(cv::Mat &mat)
{
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            cv::Vec4b& rgba = mat.at<cv::Vec4b>(i, j);
            rgba[0] = UCHAR_MAX;
            rgba[1] = cv::saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX);
            rgba[2] = cv::saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX);
            rgba[3] = cv::saturate_cast<uchar>(0.5 * (rgba[1] + rgba[2]));
        }
    }
}


void cmdCallback(redisAsyncContext *c, void *r, void *privdata) {
    redisReply *reply = (redisReply *)r;
    if (reply == NULL) {
		printf("no reply\n");
	} else {
		if(reply->str) {
			json_error_t error;
			json_t *js = json_loads(reply->str, JSON_DECODE_ANY, &error);
			if (js == NULL) {
				printf("Error JSON decoding:%s", error.text);
			} else {
				char *jstr = json_dumps(js, JSON_INDENT(4));
				if(jstr) {
					printf("%s\n", jstr);
					free(jstr);
				} else {
					printf("Can not decode JSON\n");
				}
				json_decref(js);
			}
		}
	}

    /* Disconnect after receiving the reply to GET */
//    redisAsyncDisconnect(c);
    redisAsyncCommand(redis, cmdCallback, NULL, "LPOP %s", r_cam_cmd);
}


void connectCallback(const redisAsyncContext *c) {
    ((void)c);
    printf("connected...\n");
}

void disconnectCallback(const redisAsyncContext *c, int status) {
    if (status != REDIS_OK) {
        printf("Error: %s\n", c->errstr);
    }
    printf("disconnected...\n");
}


int main (int argc, char **argv) {
    signal(SIGPIPE, SIG_IGN);
    base = event_base_new();

    redis = redisAsyncConnect("localhost", 6379);
    if (redis->err) {
        /* Let *redis leak for now... */
        printf("Error: %s\n", redis->errstr);
        return 1;
    }
	setup();
	cam_begin();
    redisLibeventAttach(redis,base);
    redisAsyncSetConnectCallback(redis,connectCallback);
    redisAsyncSetDisconnectCallback(redis,disconnectCallback);
    redisAsyncCommand(redis, NULL, NULL, "SET temp.a %b", argv[argc-1], strlen(argv[argc-1]));
    redisAsyncCommand(redis, cmdCallback, NULL, "LPOP %s", r_cam_cmd);
	do {
    	event_base_loop(base, EVLOOP_NONBLOCK);
//		loop();
    } while(!exiting);
    event_base_dispatch(base);
	cam_end();
    return 0;
}

