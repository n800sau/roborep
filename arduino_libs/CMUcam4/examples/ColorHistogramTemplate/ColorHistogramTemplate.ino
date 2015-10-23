/***************************************************************************//**
* @file
* Color Histogram Template Code
*
* @version @n 1.0
* @date @n 8/14/2012
*
* @authors @n Kwabena W. Agyeman
* @copyright @n (c) 2012 Kwabena W. Agyeman
* @n All rights reserved - Please see the end of the file for the terms of use
*
* @par Update History:
* @n v1.0 - Initial Release - 8/14/2012
*******************************************************************************/

#include <CMUcam4.h>
#include <CMUcom4.h>

#define HISTOGRAM_CHANNEL CMUCAM4_GREEN_CHANNEL
#define HISTOGRAM_BINS CMUCAM4_H32_BINS

#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);

void setup()
{
  cam.begin();

  // Wait for auto gain and auto white balance to run.

  cam.LEDOn(LED_BLINK);
  delay(WAIT_TIME);

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);

  cam.LEDOn(CMUCAM4_LED_ON);
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

/***************************************************************************//**
* @file
* @par MIT License - TERMS OF USE:
* @n Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* @n
* @n The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* @n
* @n THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/
