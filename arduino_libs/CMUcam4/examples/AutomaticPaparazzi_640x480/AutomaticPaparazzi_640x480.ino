/***************************************************************************//**
* @file
* Automatic Paparazzi Demo using 640 x 480 resolution.
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

#define RED_TOLERANCE 30
#define GREEN_TOLERANCE 30
#define BLUE_TOLERANCE 30

#define LED_BLINK 2 // 2 Hz
#define LED_SETUP 5 // 5 Hz
#define POSE_TIME 5000 // 5 seconds
#define WAIT_TIME 5000 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);

void setup()
{
  cam.begin();

  // Wait for auto gain and auto white balance to run.

  cam.LEDOn(LED_SETUP);
  delay(WAIT_TIME);

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);

  cam.LEDOn(CMUCAM4_LED_ON);
}

void loop()
{
  // Data structures.
  CMUcam4_statistics_data_t base;
  CMUcam4_statistics_data_t sample;

  // Wait for the scene to settle.
  cam.LEDOn(LED_SETUP);
  delay(WAIT_TIME);

  // Start "getMean" mode streaming.
  cam.LEDOn(CMUCAM4_LED_ON);
  cam.getMean();

  // Capture base line statisitcs.
  cam.getTypeSDataPacket(&base);

  do
  {
    // Continously capture image statistics.
    cam.getTypeSDataPacket(&sample);
  }
  while
  (
    // Look for any difference in means.
    (abs(base.RMean - sample.RMean) < RED_TOLERANCE) &&
    (abs(base.GMean - sample.GMean) < GREEN_TOLERANCE) &&
    (abs(base.BMean - sample.BMean) < BLUE_TOLERANCE)
  );

  // Something changed.
  cam.idleCamera();
  cam.LEDOn(LED_BLINK);
  delay(POSE_TIME);

  // So take a picture.
  cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480);
  cam.unmountDisk();
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
