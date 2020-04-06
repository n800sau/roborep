/***************************************************************************//**
* @file
* %CMUcam4 Hardware Acceptance Test
*
* @version @n 1.0
* @date @n 8/22/2012
*
* @authors @n Kwabena W. Agyeman
* @copyright @n (c) 2012 Kwabena W. Agyeman
* @n All rights reserved - Please see the end of the file for the terms of use
*
* @par Update History:
* @n v1.0 - Initial Release - 8/22/2012
*******************************************************************************/

#if (!defined(__AVR_ATmega1280__)) && (!defined(__AVR_ATmega2560__))
#error "Error: This test is designed to run on the Arduino Mega only!"
#endif

#include <CMUcam4.h>
#include <CMUcom4.h>

#define TEST_PORT CMUCOM4_SERIAL3

#define TEST_BAUD 19200 // Startup baud rate.
#define TEST_WAIT 5000 // Startup wait time in milliseconds.

/*******************************************************************************
* Error Code
*******************************************************************************/

#define E(func) (_E((func), __LINE__))

long _E(long func, unsigned long line)
{
  if(func < CMUCAM4_RETURN_SUCCESS)
  {
    Serial.print(F("\r\n\r\nCaught error "));
    Serial.print(func);
    Serial.print(F(" on line "));
    Serial.print(line);
    Serial.println(F(". Press enter to continue: "));
    while(Serial.read() != '\r');
  }

  return func;
}

/*******************************************************************************
* Testing Code
*******************************************************************************/

#define B_P_WAIT        2500 // Button pressed wait time.
#define B_R_WAIT        2500 // Button released wait time.
#define LED_WAIT        2000

CMUcam4 cam(TEST_PORT);

int buffer;
int counter;

CMUcam4_histogram_data_1_t type_h_1;
CMUcam4_histogram_data_2_t type_h_2;
CMUcam4_histogram_data_4_t type_h_4;
CMUcam4_histogram_data_8_t type_h_8;
CMUcam4_histogram_data_16_t type_h_16;
CMUcam4_histogram_data_32_t type_h_32;
CMUcam4_histogram_data_64_t type_h_64;

CMUcam4_statistics_data_t type_s;
CMUcam4_tracking_data_t type_t;

void setup()
{
  Serial.begin(TEST_BAUD);
  delay(TEST_WAIT);

  Serial.print(F("\r\n****************************************"));
  Serial.print(F("\r\nHardware Acceptance Test Initializing..."));
  Serial.print(F("\r\n****************************************"));

/* Begin Test *****************************************************************/
  Serial.print(F("\r\nTesting Begin... ***********************"));

  E(cam.begin());

/* Reset System Test **********************************************************/
  Serial.print(F("\r\nTesting Reset System... ****************"));

  E(cam.sleepDeeply());
  E(cam.resetSystem());
  E(cam.sleepLightly());
  E(cam.resetSystem());

/* Format Disk test ***********************************************************/
  Serial.print(F("\r\nTesting Format Disk... *****************"));

  Serial.print(F("\r\n\r\nPress enter to format the disk: "));
  while(Serial.read() != '\r');
  E(cam.formatDisk());

/* Auto Gain Control Test *****************************************************/
  Serial.print(F("\r\nTesting Auto Gain Control... ***********"));

  E(cam.autoGainControl(false));

  if((E(cam.cameraRegisterRead(0x13)) & 0x04) != 0x00)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.autoGainControl(true));

  if((E(cam.cameraRegisterRead(0x13)) & 0x04) != 0x04)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Auto White Balance Test ****************************************************/
  Serial.print(F("\r\nTesting Auto White Balance... **********"));

  E(cam.autoWhiteBalance(false));

  if((E(cam.cameraRegisterRead(0x13)) & 0x02) != 0x00)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.autoWhiteBalance(true));

  if((E(cam.cameraRegisterRead(0x13)) & 0x02) != 0x02)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Test Mode Test *************************************************************/
  Serial.print(F("\r\nTesting Test Mode... *******************"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.testMode(false));
  E(cam.colorTracking(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.testMode(true));
  E(cam.colorTracking(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

/* Color Tracking Test ********************************************************/
  Serial.print(F("\r\nTesting Color Tracking... **************"));

  E(cam.testMode(false));
  E(cam.colorTracking(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.testMode(true));
  E(cam.colorTracking(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

  E(cam.testMode(false));
  E(cam.colorTracking(false));

/* Dump Bitmap Test ***********************************************************/
  Serial.print(F("\r\nTesting Dump Bitmap... *****************"));

  E(cam.dumpBitmap());

  // Sleep mode on.
  E(cam.cameraRegisterWrite(0x13, 0xE0, 0xFF));
  E(cam.cameraRegisterWrite(0x3B, 0x06, 0x06));
  E(cam.cameraRegisterWrite(0xD5, 0x00, 0xFF));
  E(cam.cameraRegisterWrite(0xD6, 0x00, 0xFF));
  E(cam.cameraRegisterWrite(0x09, 0x10, 0x10));

  delay(1000);

  if(cam.dumpBitmap() != CMUCAM4_CAMERA_TIMEOUT_ERROR)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  // Sleep mode off.
  E(cam.cameraRegisterWrite(0x09, 0x00, 0x10));
  E(cam.cameraRegisterWrite(0xD6, 0x3F, 0xFF));
  E(cam.cameraRegisterWrite(0xD5, 0xFF, 0xFF));
  E(cam.cameraRegisterWrite(0x3B, 0x00, 0x06));
  E(cam.cameraRegisterWrite(0x13, 0xE7, 0xFF));

  delay(1000);

  E(cam.dumpBitmap());

/* Unmount Disk Test **********************************************************/
  Serial.print(F("\r\nTesting Unmount Disk... ****************"));

  E(cam.unmountDisk());

/* Get Button Pressed Test ****************************************************/
  Serial.print(F("\r\nTesting Get Button Pressed... **********"));

  E(cam.getButtonPressed());
  Serial.print(F("\r\n\r\nPress and hold the user button: "));
  while(E(cam.getButtonPressed()) != true);

/* Get Button State Test ******************************************************/
  Serial.print(F("\r\n\r\nTesting Get Button State... ************"));

  if(E(cam.getButtonState()) != true)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if(E(cam.getButtonDuration()) >= B_R_WAIT)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  while(E(cam.getButtonDuration()) < B_P_WAIT);

/* Get Button Released Test ***************************************************/
  Serial.print(F("\r\nTesting Get Button Released... *********"));

  E(cam.getButtonReleased());
  Serial.print(F("\r\n\r\nRelease the user button: "));
  while(E(cam.getButtonReleased()) != true);

/* Get Button Duration Test ***************************************************/
  Serial.print(F("\r\n\r\nTesting Get Button Duration... *********"));

  if(E(cam.getButtonState()) != false)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if(E(cam.getButtonDuration()) >= B_P_WAIT)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  while(E(cam.getButtonDuration() < B_R_WAIT));

/* LED Off and LED On Test ****************************************************/
  Serial.print(F("\r\nTesting LED Off and LED On... **********"));

  E(cam.LEDOff());
  delay(LED_WAIT);

  E(cam.LEDOn(0));
  delay(LED_WAIT);
  E(cam.LEDOn(1));
  delay(LED_WAIT);
  E(cam.LEDOn(2));
  delay(LED_WAIT);
  E(cam.LEDOn(4));
  delay(LED_WAIT);
  E(cam.LEDOn(8));
  delay(LED_WAIT);
  E(cam.LEDOn(16));
  delay(LED_WAIT);
  E(cam.LEDOn(32));
  delay(LED_WAIT);
  E(cam.LEDOn(64));
  delay(LED_WAIT);

  E(cam.LEDOn(CMUCAM4_LED_OFF));
  delay(LED_WAIT);
  E(cam.LEDOn(CMUCAM4_LED_ON));

/* Set & Get Servo Position Test **********************************************/
  Serial.print(F("\r\nTesting Set & Get Servo Position... ****"));

  for(counter = 0; counter < 359; counter++)
  {
    buffer = (((int) (cos((((float) counter) * PI) / 180.0) * 750.0)) + 1500);
    E(cam.setServoPosition(CMUCAM4_PAN_SERVO, true, buffer));

    if(E(cam.getServoPosition(CMUCAM4_PAN_SERVO)) != buffer)
    {
      E(CMUCAM4_RETURN_FAILURE);
    }

    buffer = (((int) (sin((((float) counter) * PI) / 180.0) * 750.0)) + 1500);
    E(cam.setServoPosition(CMUCAM4_TILT_SERVO, true, buffer));

    if(E(cam.getServoPosition(CMUCAM4_TILT_SERVO)) != buffer)
    {
      E(CMUCAM4_RETURN_FAILURE);
    }
  }

  E(cam.setServoPosition(CMUCAM4_PAN_SERVO, false, 0));
  E(cam.setServoPosition(CMUCAM4_TILT_SERVO, false, 0));

/* Track Color Test ***********************************************************/
  Serial.print(F("\r\nTesting Track Color and Window... ******"));

  E(cam.trackColor(0, 255, 0, 255, 0, 255));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeTDataPacket(&type_t));

    if((type_t.mx < 79) || (80 < type_t.mx) ||
    (type_t.my < 59) || (60 < type_t.my) ||
    (type_t.x1 != 0) ||
    (type_t.y1 != 0) ||
    (type_t.x2 != 159) ||
    (type_t.y2 != 119) ||
    (type_t.pixels != 255) ||
    (type_t.confidence != 255))
    {
      E(CMUCAM4_RETURN_FAILURE);
    }
  }

  E(cam.testMode(true));

  for(buffer = 0; buffer < 8; buffer++)
  {
    E(cam.setTrackingWindow(buffer * 20, 0, ((buffer + 1) * 20) - 1, 119));
    E(cam.trackWindow(20, 20, 20));
    E(cam.setTrackingWindow());
    E(cam.trackColor());

    for(counter = 0; counter < 30; counter++)
    {
      E(cam.getTypeTDataPacket(&type_t));

      if((type_t.mx < (6+(buffer*20))) || ((13+(buffer*20)) < type_t.mx) ||
      (type_t.my < 58) || (61 < type_t.my) ||
      (type_t.x1 < ((buffer*20)-3)) || (((buffer*20)+4) < type_t.x1) ||
      (type_t.y1 < 0) || (1 < type_t.y1) ||
      (type_t.x2 < ((buffer*20)+16)) || (((buffer*20)+23) < type_t.x2) ||
      (type_t.y2 < 118) || (119 < type_t.y2) ||
      (type_t.pixels < 28) || (35 < type_t.pixels) ||
      (type_t.confidence < 251) || (255 < type_t.confidence))
      {
        E(CMUCAM4_RETURN_FAILURE);
      }
    }
  }

  E(cam.testMode(false));

/* Get Histogram Test *********************************************************/
  Serial.print(F("\r\nTesting Get Histogram... ***************"));

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H1_BINS));
  E(cam.getTypeHDataPacket(&type_h_1));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_1_T_LENGTH; counter++)
  {
    buffer += type_h_1.bins[counter];
  }

  if((buffer < 255) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H2_BINS));
  E(cam.getTypeHDataPacket(&type_h_2));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_2_T_LENGTH; counter++)
  {
    buffer += type_h_2.bins[counter];
  }

  if((buffer < 254) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H4_BINS));
  E(cam.getTypeHDataPacket(&type_h_4));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_4_T_LENGTH; counter++)
  {
    buffer += type_h_4.bins[counter];
  }

  if((buffer < 252) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H8_BINS));
  E(cam.getTypeHDataPacket(&type_h_8));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_8_T_LENGTH; counter++)
  {
    buffer += type_h_8.bins[counter];
  }

  if((buffer < 248) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H16_BINS));
  E(cam.getTypeHDataPacket(&type_h_16));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_16_T_LENGTH; counter++)
  {
    buffer += type_h_16.bins[counter];
  }

  if((buffer < 240) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H32_BINS));
  E(cam.getTypeHDataPacket(&type_h_32));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_32_T_LENGTH; counter++)
  {
    buffer += type_h_32.bins[counter];
  }

  if((buffer < 224) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_64) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H64_BINS));
  E(cam.getTypeHDataPacket(&type_h_64));

  for(buffer = 0, counter = 0; counter < CMUCAM4_HD_64_T_LENGTH; counter++)
  {
    buffer += type_h_64.bins[counter];
  }

  if((buffer < 192) || (255 < buffer))
  { E(CMUCAM4_RETURN_FAILURE); }

  if(cam.getTypeHDataPacket(&type_h_1) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_2) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_4) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_8) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_16) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }
  if(cam.getTypeHDataPacket(&type_h_32) != CMUCAM4_UNEXPECTED_RESPONCE)
  { E(CMUCAM4_RETURN_FAILURE); }

/* Get Mean Test **************************************************************/
  Serial.print(F("\r\nTesting Get Mean... ********************"));

  E(cam.testMode(true));

  E(cam.getMean());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeSDataPacket(&type_s));

    // S 156 141 156 115 60 156 255 255 255 90 109 98
    // S 156 141 156 123 60 156 255 255 255 90 109 98

    if((type_s.RMean < 106) || (206 < type_s.RMean) ||
    (type_s.GMean < 91) || (191 < type_s.GMean) ||
    (type_s.BMean < 106) || (206 < type_s.BMean) ||
    (type_s.RStDev < 40) || (140 < type_s.RStDev) ||
    (type_s.GStDev < 59) || (159 < type_s.GStDev) ||
    (type_s.BStDev < 48) || (148 < type_s.BStDev))
    {
      E(CMUCAM4_RETURN_FAILURE);
    }
  }

  E(cam.testMode(false));

/* End Test *******************************************************************/
  Serial.print(F("\r\nTesting End... *************************"));

  E(cam.end());

/******************************************************************************/

  Serial.print(F("\r\n****************************************"));
  Serial.print(F("\r\nHardware Acceptance Test Finalizing....."));
  Serial.print(F("\r\n****************************************"));

  return;
}

void loop()
{
  return;
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
