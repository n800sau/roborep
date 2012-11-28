#include <unistd.h>
#include <time.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#include <vector>
#include <iostream>
#include <string>

#include "CMUcam4.h"
#include "CMUcom4.h"

#define RED_MIN 20
#define RED_MAX 255
#define GREEN_MIN 23
#define GREEN_MAX 255
#define BLUE_MIN 23
#define BLUE_MAX 255

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);

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
	cam.monitorOn();
	//cam.automaticPan(1, 0);
	cam_end();
}


void loop()
{




//  for(;;)
//  {
//    cam.getTypeTDataPacket(&t_data); // Get a tracking packet.
//    cam.getTypeFDataPacket(&f_data); // Get an image packet.

    // Process the packet data safely here.
//  }

  // Do something else here.
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


void printH(int tilt, int p, const uint8_t *bins, int length)
{
	FILE *f = fopen("bins.txt", "a");
	if(f) {
		time_t t = time(NULL);
		struct tm *st = localtime(&t);
		fprintf(f, "%.4d.%2d.%2d %.2d:%.2d:%.2d,%d,%d", st->tm_year+1900, st->tm_mon+1, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec, tilt, p);
		for(int i = 0; i< length; i++) {
			fprintf(f, ",%d", bins[i]);
		}
		fprintf(f, "\n");
		fclose(f);
	} else {
		printf("Can not open log file\n");
		exit(1);
	}
}


void start()
{
	CMUcam4_image_data_t f_data;
	CMUcam4_tracking_data_t t_data;
	setup();
//	cam.lineMode(true);
//	cam.automaticPan(1, 0);
//	cam.testMode(1);
	cam_begin();

#define HISTOGRAM_CHANNEL CMUCAM4_GREEN_CHANNEL
#define HISTOGRAM_BINS CMUCAM4_H4_BINS
  CMUcam4_histogram_data_1_t h_1_data;
  CMUcam4_histogram_data_2_t h_2_data;
  CMUcam4_histogram_data_4_t h_4_data;
  CMUcam4_histogram_data_8_t h_8_data;
  CMUcam4_histogram_data_16_t h_16_data;
  CMUcam4_histogram_data_32_t h_32_data;
  CMUcam4_histogram_data_64_t h_64_data;

	//printf("colorT=%d\n", cam.colorTracking(1));
	int servo_steps = 2250 - 750;
	for(int t = 0; t < servo_steps/2 ; t += 40) {
		cam.setServoPosition(CMUCAM4_TILT_SERVO, true, t+750);
		for(int p = 0; p < servo_steps; p += 40) {
			printf("\nt=%d, p=%d:\n", t, p);
			cam.setServoPosition(CMUCAM4_PAN_SERVO, true, p+750);
			usleep(100000L);
			printf("histogram=%d\n", cam.getHistogram(HISTOGRAM_CHANNEL, HISTOGRAM_BINS));
			uint16_t buffer[80*80]; // 10 by 10 pixel buffer
			uint8_t buffer3[80*80][3]; // 10 by 10 pixel buffer
			memset(buffer, 0, sizeof(buffer));
/*			printf("sendFrame=%d\n", cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_HR_80, buffer, 80, 0, 80, 0));

			char fname[256];
			sprintf(fname, "%d.%d.jpg", t, p);
	
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

			int p[3] = {CV_IMWRITE_JPEG_QUALITY, 10, 0};
			cvSaveImage(fname, cv_image, p);
			printf("end\n");

			// release resources
			cvReleaseImageHeader(&cv_image);
*/

    #if(HISTOGRAM_BINS == CMUCAM4_H1_BINS)
      cam.getTypeHDataPacket(&h_1_data); // Get a histogram packet.
		printH(t, p, h_1_data.bins, CMUCAM4_HD_1_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H2_BINS)
      cam.getTypeHDataPacket(&h_2_data); // Get a histogram packet.
		printH(t, p, h_2_data.bins, CMUCAM4_HD_2_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H4_BINS)
      cam.getTypeHDataPacket(&h_4_data); // Get a histogram packet.
		printH(t, p, h_4_data.bins, CMUCAM4_HD_4_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H8_BINS)
      cam.getTypeHDataPacket(&h_8_data); // Get a histogram packet.
		printH(t, p, h_8_data.bins, CMUCAM4_HD_8_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H16_BINS)
      cam.getTypeHDataPacket(&h_16_data); // Get a histogram packet.
		printH(t, p, h_16_data.bins, CMUCAM4_HD_16_T_LENGTH);

    #elif(HISTOGRAM_BINS == CMUCAM4_H32_BINS)
      cam.getTypeHDataPacket(&h_32_data); // Get a histogram packet.
		printH(t, p, h_32_data.bins, CMUCAM4_HD_64_T_LENGTH);

    #elif((HISTOGRAM_BINS == CMUCAM4_H64_BINS) && \
    (HISTOGRAM_CHANNEL == CMUCAM4_GREEN_CHANNEL))
      cam.getTypeHDataPacket(&h_64_data); // Get a histogram packet.
		printH(t, p, h_64_data.bins, CMUCAM4_HD_64_T_LENGTH);

    #else // 1 to 64 bins...
      #error "Invalid number of bins"

    #endif



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
		}
	}
	cam_end();
//	cam.automaticPan(0, 0);
}

int main(int argc, const char **argv)
{
	start();
	return 0;
}

