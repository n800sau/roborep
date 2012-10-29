#include <unistd.h>
#include "CMUcam4.h"
#include "CMUcom4.h"

#define HISTOGRAM_CHANNEL CMUCAM4_GREEN_CHANNEL
#define HISTOGRAM_BINS CMUCAM4_H32_BINS

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);

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
	while(true) {
		loop();
	}
}

int main(int argc, const char **argv)
{
	start();
	return 0;
}

