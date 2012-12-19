#include <unistd.h>
#include <time.h>

#include "CMUcam4.h"
#include "CMUcom4.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define HISTOGRAM_CHANNEL CMUCAM4_GREEN_CHANNEL
#define HISTOGRAM_BINS CMUCAM4_H32_BINS

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5 // 5 seconds

#define E(func) (_E((func), __LINE__))

long _E(long func, unsigned long line)
{
  if(func < CMUCAM4_RETURN_SUCCESS)
  {
    printf("\nCaught error %d on line %d\n", func, line);
  }

  return func;
}

CMUcam4 cam(CMUCOM4_SERIAL);

struct IMBUF {
	int isize;
	uint8_t ibuf[160*120*4];
};

void clearSD()
{
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
}

int getImage(IMBUF &ibuf)
{
	clearSD();
#define DE_SIZE 4
	ibuf.isize = 0;
	CMUcam4_directory_entry_t de[DE_SIZE]; // Directory entry array.
	if(E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120)) == 0) {
		int directorySize = E(cam.listDirectory(de, DE_SIZE, 0));
		if(directorySize > 0) {
			const char *fname = de[directorySize - 1].name;
			printf("File name:%s\n", fname);
			ibuf.isize = cam.filePrint(fname, ibuf.ibuf, sizeof(ibuf.ibuf), 0);
			if(ibuf.isize < 0) {
				printf("Error reading file %s\n", fname);
			}
		}
	}
	return ibuf.isize;
}

void loop()
{
  CMUcam4_histogram_data_1_t h_1_data;
  CMUcam4_histogram_data_2_t h_2_data;
  CMUcam4_histogram_data_4_t h_4_data;
  CMUcam4_histogram_data_8_t h_8_data;
  CMUcam4_histogram_data_16_t h_16_data;
  CMUcam4_histogram_data_32_t h_32_data;
  CMUcam4_histogram_data_64_t h_64_data;

  cam.getHistogram(HISTOGRAM_CHANNEL, HISTOGRAM_BINS);

  for(;;)
  {
    #if(HISTOGRAM_BINS == CMUCAM4_H1_BINS)
      cam.getTypeHDataPacket(&h_1_data); // Get a histogram packet.

    #elif(HISTOGRAM_BINS == CMUCAM4_H2_BINS)
      cam.getTypeHDataPacket(&h_2_data); // Get a histogram packet.

    #elif(HISTOGRAM_BINS == CMUCAM4_H4_BINS)
      cam.getTypeHDataPacket(&h_4_data); // Get a histogram packet.

    #elif(HISTOGRAM_BINS == CMUCAM4_H8_BINS)
      cam.getTypeHDataPacket(&h_8_data); // Get a histogram packet.

    #elif(HISTOGRAM_BINS == CMUCAM4_H16_BINS)
      cam.getTypeHDataPacket(&h_16_data); // Get a histogram packet.

    #elif(HISTOGRAM_BINS == CMUCAM4_H32_BINS)
      cam.getTypeHDataPacket(&h_32_data); // Get a histogram packet.

    #elif((HISTOGRAM_BINS == CMUCAM4_H64_BINS) && \
    (HISTOGRAM_CHANNEL == CMUCAM4_GREEN_CHANNEL))
      cam.getTypeHDataPacket(&h_64_data); // Get a histogram packet.

    #else // 1 to 64 bins...
      #error "Invalid number of bins"

    #endif

    // Process the packet data.
  }

  // Do something else here.
}

void start()
{
  cam.begin();

  // Wait for auto gain and auto white balance to run.

  cam.LEDOn(LED_BLINK);
  sleep(WAIT_TIME);

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);

  cam.LEDOn(CMUCAM4_LED_ON);
//	while(true) {
//		loop();
//	}
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

int main(int argc, const char **argv)
{
	start();
	IMBUF ibuf;
	getImage(ibuf);
	FILE *df = fopen("img.bmp", "wb");
	if(df) {
		fwrite(ibuf.ibuf, 1, ibuf.isize, df);
		fclose(df);
	}
	cv::Mat M(120, 160, CV_16UC3, ibuf.ibuf);


	printf("col:%d,rows:%d\n", M.cols, M.rows);
//	createAlphaMat(M);
//	printf("here1\n");


			
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);

/*			printf("here2\n");
			for(int y = 0; y < 40 ; y++) {
				for(int x = 0; x < 40 ; x++) {
					printf("%4.4X", (int)M.data[x+y*80]);
				}
			}*/
			imwrite("img.png", M, compression_params);
			printf("here3\n");

	return 0;
}

