/***************************************************************************//**
* @file
* %CMUcam4 Software Acceptance Test
*
* @version @n 1.0
* @date @n 8/12/2012
*
* @authors @n Kwabena W. Agyeman
* @copyright @n (c) 2012 Kwabena W. Agyeman
* @n All rights reserved - Please see the end of the file for the terms of use
*
* @par Update History:
* @n v1.0 - Initial Release - 8/12/2012
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

#define FE_SIZE         256 // File entry array size.
#define DE_SIZE         4 // Directory entry array size.

#define DOT_INDEX       0 // The "." entry position.
#define DOT_DOT_INDEX   1 // The ".." entry position.
#define TXT_TXT_INDEX   2
#define ENTRY_COUNT     3

#define B_P_WAIT        2500 // Button pressed wait time.
#define B_R_WAIT        2500 // Button released wait time.
#define LED_WAIT        2000
#define MONITOR_WAIT    5000

CMUcam4 cam(TEST_PORT);

CMUcam4_disk_information_t di;
CMUcam4_disk_space_t ds;

char fe[FE_SIZE]; // File entry array.
CMUcam4_directory_entry_t de[DE_SIZE]; // Directory entry array.

long fileSize;
long directorySize;

int buffer;
int counter;

int row;
int column;

const char * fileName = "TEXTFILE.TXT";
const char * folderName = "FOLDER";

const char * oldFileName = "/TEXTFILE.TXT";
const char * newFileName = "TXT.TXT";

const char * pangram0 = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG";
const char * pangram1 = "1234567890";
const char * pangram2 = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG"
                        "\r1234567890\r";
const char * pangram3 = "\r1234567890\r";

const char * attribRH__SA = "RHS__A";
const char * attrib______ = "______";

CMUcam4_tracking_parameters_t tp;
CMUcam4_tracking_window_t tw;

CMUcam4_image_data_t type_f, type_f_copy0, type_f_copy1;
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
  Serial.print(F("\r\nSoftware Acceptance Test Initializing..."));
  Serial.print(F("\r\n****************************************"));

/* Begin Test *****************************************************************/
  Serial.print(F("\r\nTesting Begin... ***********************"));

  E(cam.begin());

/* Get Version Test ***********************************************************/
  Serial.print(F("\r\nTesting Get Version... *****************"));

  Serial.print(F("\r\n\r\nFirmware Version: "));
  Serial.println(E(cam.getVersion()));

/* Reset System Test **********************************************************/
  Serial.print(F("\r\nTesting Reset System... ****************"));

  E(cam.sleepDeeply());
  E(cam.resetSystem());
  E(cam.sleepLightly());
  E(cam.resetSystem());

/* Sleep Deeply Test **********************************************************/
  Serial.print(F("\r\nTesting Sleep Deeply... ****************"));

  E(cam.sleepDeeply());
  E(cam.idleCamera());
  E(cam.sleepDeeply());
  E(cam.idleCamera());

/* Sleep Deeply Test **********************************************************/
  Serial.print(F("\r\nTesting Sleep Lightly... ***************"));

  E(cam.sleepLightly());
  E(cam.idleCamera());
  E(cam.sleepLightly());
  E(cam.idleCamera());

/* Format Disk test ***********************************************************/
  Serial.print(F("\r\nTesting Format Disk... *****************"));

  Serial.print(F("\r\n\r\nPress enter to format the disk: "));
  while(Serial.read() != '\r');
  E(cam.formatDisk());

/* Disk Information Test ******************************************************/
  Serial.print(F("\r\n\r\nTesting Disk Information... ************"));

  E(cam.diskInformation(&di));

  Serial.print(F("\r\n\r\nVolume Label: "));
  Serial.print(di.volumeLabel);
  Serial.print(F("\r\nFile System Type: "));
  Serial.print(di.fileSystemType);
  Serial.print(F("\r\nDisk Signature: 0x"));
  Serial.print(di.diskSignature, HEX);
  Serial.print(F("\r\nVolume Indentification: 0x"));
  Serial.print(di.volumeIdentification, HEX);
  Serial.print(F("\r\nCount of Data Sectors: "));
  Serial.print(di.countOfDataSectors);
  Serial.print(F("\r\nBytes per Sector: "));
  Serial.print(di.bytesPerSector);
  Serial.print(F("\r\nSectors per Cluster: "));
  Serial.print(di.sectorsPerCluster);
  Serial.print(F("\r\nCount of Clusters: "));
  Serial.println(di.countOfClusters);

  if(di.countOfDataSectors != (di.sectorsPerCluster * di.countOfClusters))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Disk Space Test ************************************************************/
  Serial.print(F("\r\nTesting Disk Space... ******************"));

  E(cam.diskSpace(&ds));

  Serial.print(F("\r\n\r\nFree Sector Count: "));
  Serial.print(ds.freeSectorCount);
  Serial.print(F("\r\nUsed Sector Count: "));
  Serial.println(ds.usedSectorCount);

  if(di.countOfDataSectors != (ds.freeSectorCount + ds.usedSectorCount))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Print Line Test ************************************************************/
  Serial.print(F("\r\nTesting Print Line... ******************"));

  E(cam.printLine(fileName, pangram0));
  E(cam.printLine(fileName, pangram1));

/* File Print Test ************************************************************/
  Serial.print(F("\r\nTesting File Print... ******************"));

  fileSize = E(cam.filePrint(fileName, (uint8_t *) &fe, FE_SIZE, 0));

  if(fileSize != strlen(pangram2))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if(memcmp(fe, pangram2, fileSize) != 0)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.filePrint(fileName, (uint8_t *) &fe, FE_SIZE, strlen(pangram0)));

  if(memcmp(fe, pangram3, strlen(pangram3)) != 0)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Make Directory Test ********************************************************/
  Serial.print(F("\r\nTesting Make Directory... **************"));

  E(cam.makeDirectory(folderName));

/* Change Directory Test ******************************************************/
  Serial.print(F("\r\nTesting Change Directory... ************"));

  E(cam.changeDirectory(folderName));

/* Move Entry Test ************************************************************/
  Serial.print(F("\r\nTesting Move Entry... ******************"));

  E(cam.moveEntry(oldFileName, newFileName));

/* Change Attributes Test *****************************************************/
  Serial.print(F("\r\nTesting Change Attributes... ***********"));

  E(cam.changeAttributes(newFileName, attribRH__SA));

/* List Directory Test ********************************************************/
  Serial.print(F("\r\nTesting List Directory... **************"));

  directorySize = E(cam.listDirectory(de, DE_SIZE, 0));

  if(directorySize != ENTRY_COUNT)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  Serial.print(F("\r\n\r\nName: "));
  Serial.print(de[DOT_INDEX].name);
  Serial.print(F("\r\nAttributes: "));
  Serial.print(de[DOT_INDEX].attributes);
  Serial.print(F("\r\nSize: "));
  Serial.print(de[DOT_INDEX].size);

  Serial.print(F("\r\nName: "));
  Serial.print(de[DOT_DOT_INDEX].name);
  Serial.print(F("\r\nAttributes: "));
  Serial.print(de[DOT_DOT_INDEX].attributes);
  Serial.print(F("\r\nSize: "));
  Serial.print(de[DOT_DOT_INDEX].size);

  Serial.print(F("\r\nName: "));
  Serial.print(de[TXT_TXT_INDEX].name);
  Serial.print(F("\r\nAttributes: "));
  Serial.print(de[TXT_TXT_INDEX].attributes);
  Serial.print(F("\r\nSize: "));
  Serial.println(de[TXT_TXT_INDEX].size);

  if((strstr(de[TXT_TXT_INDEX].name, newFileName) == NULL) ||
  (strstr(de[TXT_TXT_INDEX].attributes, attribRH__SA) == NULL) ||
  (de[TXT_TXT_INDEX].size != fileSize))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if((cam.isReadOnly(&(de[DOT_INDEX])) == true) ||
  (cam.isHidden(&(de[DOT_INDEX])) == true) ||
  (cam.isSystem(&(de[DOT_INDEX])) == true) ||
  (cam.isVolumeID(&(de[DOT_INDEX])) == true) ||
  (cam.isDirectory(&(de[DOT_INDEX])) == false) ||
  (cam.isArchive(&(de[DOT_INDEX])) == true))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if((cam.isReadOnly(&(de[DOT_DOT_INDEX])) == true) ||
  (cam.isHidden(&(de[DOT_DOT_INDEX])) == true) ||
  (cam.isSystem(&(de[DOT_DOT_INDEX])) == true) ||
  (cam.isVolumeID(&(de[DOT_DOT_INDEX])) == true) ||
  (cam.isDirectory(&(de[DOT_DOT_INDEX])) == false) ||
  (cam.isArchive(&(de[DOT_DOT_INDEX])) == true))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if((cam.isReadOnly(&(de[TXT_TXT_INDEX])) == false) ||
  (cam.isHidden(&(de[TXT_TXT_INDEX])) == false) ||
  (cam.isSystem(&(de[TXT_TXT_INDEX])) == false) ||
  (cam.isVolumeID(&(de[TXT_TXT_INDEX])) == true) ||
  (cam.isDirectory(&(de[TXT_TXT_INDEX])) == true) ||
  (cam.isArchive(&(de[TXT_TXT_INDEX])) == false))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.listDirectory(de, DE_SIZE, TXT_TXT_INDEX));

  if((strstr(de[DOT_INDEX].name, newFileName) == NULL) ||
  (strstr(de[DOT_INDEX].attributes, attribRH__SA) == NULL) ||
  (de[DOT_INDEX].size != fileSize))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  if((cam.isReadOnly(&(de[DOT_INDEX])) == false) ||
  (cam.isHidden(&(de[DOT_INDEX])) == false) ||
  (cam.isSystem(&(de[DOT_INDEX])) == false) ||
  (cam.isVolumeID(&(de[DOT_INDEX])) == true) ||
  (cam.isDirectory(&(de[DOT_INDEX])) == true) ||
  (cam.isArchive(&(de[DOT_INDEX])) == false))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Remove Entry Test **********************************************************/
  Serial.print(F("\r\nTesting Remove entry... ****************"));

  E(cam.changeAttributes(newFileName, attrib______));
  E(cam.removeEntry(newFileName));
  E(cam.changeDirectory(".."));
  E(cam.removeEntry(folderName));

/* Camera Brightness Test *****************************************************/
  Serial.print(F("\r\nTesting Camera Brightness... ***********"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.cameraBrightness(-127));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(-63));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(-31));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(-15));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(0));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(+15));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(+31));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(+63));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraBrightness(+127));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));

  E(cam.cameraBrightness(0));

/* Camera Contrast Test *******************************************************/
  Serial.print(F("\r\nTesting Camera Contrast... *************"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.cameraContrast(-31));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(-15));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(-7));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(-3));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(0));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(+3));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(+7));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(+15));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.cameraContrast(+31));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));

  E(cam.cameraContrast(0));

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

/* Horizontal Mirror Test *****************************************************/
  Serial.print(F("\r\nTesting Horizontal Mirror... ***********"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.horizontalMirror(false));
  E(cam.verticalFlip(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.horizontalMirror(true));
  E(cam.verticalFlip(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

/* Vertical Flip Test *********************************************************/
  Serial.print(F("\r\nTesting Vertical Flip... ***************"));

  E(cam.horizontalMirror(false));
  E(cam.verticalFlip(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.horizontalMirror(true));
  E(cam.verticalFlip(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

  E(cam.horizontalMirror(false));
  E(cam.verticalFlip(false));

/* Black and White Mode Test **************************************************/
  Serial.print(F("\r\nTesting Black and White Mode... ********"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.blackAndWhiteMode(false));
  E(cam.negativeMode(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.blackAndWhiteMode(true));
  E(cam.negativeMode(false));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

/* Negative Mode Test *********************************************************/
  Serial.print(F("\r\nTesting Negative Mode... ***************"));

  E(cam.blackAndWhiteMode(false));
  E(cam.negativeMode(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));
  E(cam.blackAndWhiteMode(true));
  E(cam.negativeMode(true));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

  E(cam.blackAndWhiteMode(false));
  E(cam.negativeMode(false));

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

/* Dump Frame Test ************************************************************/
  Serial.print(F("\r\nTesting Dump Frame... ******************"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.dumpFrame(CMUCAM4_HR_80, CMUCAM4_VR_60));
  E(cam.dumpFrame(CMUCAM4_HR_80, CMUCAM4_VR_120));
  E(cam.dumpFrame(CMUCAM4_HR_80, CMUCAM4_VR_240));
  E(cam.dumpFrame(CMUCAM4_HR_80, CMUCAM4_VR_480));

  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_60));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_240));
  E(cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_480));

  E(cam.dumpFrame(CMUCAM4_HR_320, CMUCAM4_VR_60));
  E(cam.dumpFrame(CMUCAM4_HR_320, CMUCAM4_VR_120));
  E(cam.dumpFrame(CMUCAM4_HR_320, CMUCAM4_VR_240));
  E(cam.dumpFrame(CMUCAM4_HR_320, CMUCAM4_VR_480));

  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_60));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_120));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_240));
  E(cam.dumpFrame(CMUCAM4_HR_640, CMUCAM4_VR_480));

/* Send Bitmap Test ***********************************************************/
  Serial.print(F("\r\nTesting Send Bitmap... *****************"));

  E(cam.testMode(true));

  E(cam.setTrackingParameters(230, 255, 230, 255, 230, 255));
  E(cam.sendBitmap(&type_f));
  E(cam.dumpBitmap());

  E(cam.xorPixels(&type_f_copy0, &type_f_copy0, &type_f_copy0));
  E(cam.orPixels(&type_f_copy0, &type_f, &type_f_copy0));

  E(cam.printLine(fileName, "WHITE BAR"));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      fe[column] = E(cam.getPixel(&type_f, row, column)) ? '1' : '0';
    }

    fe[CMUCAM4_BINARY_H_RES] = '\0';
    E(cam.printLine(fileName, fe));
  }

  E(cam.setTrackingParameters(230, 255, 0, 100, 230, 255));
  E(cam.sendBitmap(&type_f));
  E(cam.dumpBitmap());

  E(cam.xorPixels(&type_f_copy1, &type_f_copy1, &type_f_copy1));
  E(cam.orPixels(&type_f_copy1, &type_f, &type_f_copy1));

  E(cam.printLine(fileName, "MAGENTA BAR"));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      fe[column] = E(cam.getPixel(&type_f, row, column)) ? '1' : '0';
    }

    fe[CMUCAM4_BINARY_H_RES] = '\0';
    E(cam.printLine(fileName, fe));
  }

  E(cam.notPixels(&type_f_copy0));
  E(cam.notPixels(&type_f_copy1));
  E(cam.andPixels(&type_f, &type_f_copy0, &type_f_copy1));

  E(cam.printLine(fileName, "OTHER BARS"));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      fe[column] = E(cam.getPixel(&type_f, row, column)) ? '1' : '0';
    }

    fe[CMUCAM4_BINARY_H_RES] = '\0';
    E(cam.printLine(fileName, fe));
  }

  E(cam.xorPixels(&type_f, &type_f, &type_f));
  randomSeed('C' + 'M' + 'U' + 'c' + 'a' + 'm' + '4');

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      E(cam.setPixel(&type_f, row, column, random(0, 2)));
    }
  }

  E(cam.printLine(fileName, "RANDOM PIXELS"));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      fe[column] = E(cam.getPixel(&type_f, row, column)) ? '1' : '0';
    }

    fe[CMUCAM4_BINARY_H_RES] = '\0';
    E(cam.printLine(fileName, fe));
  }

  E(cam.xorPixels(&type_f, &type_f, &type_f));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      CMUCAM4_SET_PIXEL(&type_f, row, column, ((row & 1) ^ (column & 1)));
    }
  }

  E(cam.printLine(fileName, "UNRANDOM PIXELS"));

  for(row = 0; row < CMUCAM4_BINARY_V_RES; row++)
  {
    for(column = 0; column < CMUCAM4_BINARY_H_RES; column++)
    {
      fe[column] = CMUCAM4_GET_PIXEL(&type_f, row, column) ? '1' : '0';
    }

    fe[CMUCAM4_BINARY_H_RES] = '\0';
    E(cam.printLine(fileName, fe));
  }

  E(cam.testMode(false));

/* Send Frame Test ************************************************************/
  Serial.print(F("\r\nTesting Send Frame... ******************"));

  E(cam.autoGainControl(false));
  E(cam.autoWhiteBalance(false));

  E(cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_VR_60, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_VR_120, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_VR_240, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_80, CMUCAM4_VR_480, NULL, 0, 0, 0, 0));

  E(cam.sendFrame(CMUCAM4_HR_160, CMUCAM4_VR_60, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_160, CMUCAM4_VR_120, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_160, CMUCAM4_VR_240, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_160, CMUCAM4_VR_480, NULL, 0, 0, 0, 0));

  E(cam.sendFrame(CMUCAM4_HR_320, CMUCAM4_VR_60, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_320, CMUCAM4_VR_120, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_320, CMUCAM4_VR_240, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_320, CMUCAM4_VR_480, NULL, 0, 0, 0, 0));

  E(cam.sendFrame(CMUCAM4_HR_640, CMUCAM4_VR_60, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_640, CMUCAM4_VR_120, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_640, CMUCAM4_VR_240, NULL, 0, 0, 0, 0));
  E(cam.sendFrame(CMUCAM4_HR_640, CMUCAM4_VR_480, NULL, 0, 0, 0, 0));

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

/* Pan Output & Pan Input Test ************************************************/
  Serial.print(F("\r\nTesting Pan Output & Pan Input... ******"));

  E(cam.panOutput(true, true));
  if(cam.panInput() != true) { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.panOutput(true, false));
  if(cam.panInput() != false) { E(CMUCAM4_RETURN_FAILURE); }

/* Tilt Output & Tilt Input Test **********************************************/
  Serial.print(F("\r\nTesting Tilt Output & Tilt Input... ****"));

  E(cam.tiltOutput(true, true));
  if(E(cam.tiltInput() != true)) { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.tiltOutput(true, false));
  if(E(cam.tiltInput() != false)) { E(CMUCAM4_RETURN_FAILURE); }

/* Set Outputs & Get Inputs Test **********************************************/
  Serial.print(F("\r\nTesting Set Outputs & Get Inputs... ****"));

  E(cam.setOutputs(CMUCAM4_DIR_TP_OO, CMUCAM4_OUT_TP_HH));
  if(E(cam.getInputs() != CMUCAM4_IN_TP_HH)) { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.setOutputs(CMUCAM4_DIR_TP_OO, CMUCAM4_OUT_TP_LH));
  if(E(cam.getInputs() != CMUCAM4_IN_TP_LH)) { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.setOutputs(CMUCAM4_DIR_TP_OO, CMUCAM4_OUT_TP_HL));
  if(E(cam.getInputs() != CMUCAM4_IN_TP_HL)) { E(CMUCAM4_RETURN_FAILURE); }

  E(cam.setOutputs(CMUCAM4_DIR_TP_OO, CMUCAM4_OUT_TP_LL));
  if(E(cam.getInputs() != CMUCAM4_IN_TP_LL)) { E(CMUCAM4_RETURN_FAILURE); }

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

/* Automatic Pan Test *********************************************************/
  Serial.print(F("\r\nTesting Automatic Pan & Parameters... **"));

  E(cam.automaticPan(true, true));
  E(cam.automaticPan(true, false));
  E(cam.automaticPan(false, true));
  E(cam.automaticPan(false, false));

  E(cam.autoPanParameters(1000, 1000));
  E(cam.autoPanParameters(0, 0));
  E(cam.autoPanParameters(400, 100));
  E(cam.autoPanParameters(375, 125));

/* Automatic Tilt Test ********************************************************/
  Serial.print(F("\r\nTesting Automatic Tilt & Parameters... *"));

  E(cam.automaticTilt(true, true));
  E(cam.automaticTilt(true, false));
  E(cam.automaticTilt(false, true));
  E(cam.automaticTilt(false, false));

  E(cam.autoTiltParameters(1000, 1000));
  E(cam.autoTiltParameters(0, 0));
  E(cam.autoTiltParameters(400, 100));
  E(cam.autoTiltParameters(375, 125));

/* Monitor Off Test ***********************************************************/
  Serial.print(F("\r\nTesting Monitor Off... *****************"));

  E(cam.monitorOff());
  delay(MONITOR_WAIT);

/* Monitor On Test ************************************************************/
  Serial.print(F("\r\nTesting Monitor On... ******************"));

  E(cam.monitorOn());
  delay(MONITOR_WAIT);

/* Monitor Signal Test ********************************************************/
  Serial.print(F("\r\nTesting Monitor Signal... **************"));

  E(cam.monitorSignal(true));
  delay(MONITOR_WAIT);
  E(cam.monitorSignal(false));
  delay(MONITOR_WAIT);

/* Monitor Freeze Test ********************************************************/
  Serial.print(F("\r\nTesting Monitor Freeze... **************"));

  E(cam.monitorFreeze(false));
  delay(MONITOR_WAIT);
  E(cam.monitorFreeze(true));
  delay(MONITOR_WAIT);

/* Get & Set Tracking Parameters Test *****************************************/
  Serial.print(F("\r\nTesting Get & Set Tracking Params... ***"));

  // NOTE: The output is not equal to the input for all tracking parameters.

  E(cam.setTrackingParameters(230, 255, 230, 255, 230, 255));
  E(cam.getTrackingParameters(&tp));

  if((tp.redMin != 230) || (tp.redMax != 255) ||
  (tp.greenMin != 230) || (tp.greenMax != 255) ||
  (tp.blueMin != 230) || (tp.blueMax != 255))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.setTrackingParameters());
  E(cam.getTrackingParameters(&tp));

  if((tp.redMin != 0) || (tp.redMax != 255) ||
  (tp.greenMin != 0) || (tp.greenMax != 255) ||
  (tp.blueMin != 0) || (tp.blueMax != 255))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Get & Set Tracking Window Test *********************************************/
  Serial.print(F("\r\nTesting Get & Set Tracking Window... ***"));

  // NOTE: The output is equal to the input for all tracking window parameters.

  E(cam.setTrackingWindow(40, 30, 119, 89));
  E(cam.getTrackingWindow(&tw));

  if((tw.topLeftX != 40) || (tw.topLeftY != 30) ||
  (tw.bottomRightX != 119) || (tw.bottomRightY != 89))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.setTrackingWindow());
  E(cam.getTrackingWindow(&tw));

  if((tw.topLeftX != 0) || (tw.topLeftY != 0) ||
  (tw.bottomRightX != 159) || (tw.bottomRightY != 119))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

/* Idle Camera Test ***********************************************************/
  Serial.print(F("\r\nTesting Idle Camera.. ******************"));

  E(cam.trackColor());
  E(cam.idleCamera());

  E(cam.trackColor(230, 255, 230, 255, 230, 255));
  E(cam.idleCamera());

  E(cam.trackWindow(30, 30, 30));
  E(cam.idleCamera());

  E(cam.getHistogram(0, 0));
  E(cam.idleCamera());

  E(cam.getMean());
  E(cam.idleCamera());

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

/* Poll Mode Test *************************************************************/
  Serial.print(F("\r\nTesting Poll Mode... *******************"));

  E(cam.pollMode(true));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.trackColor());
    E(cam.getTypeTDataPacket(&type_t));

    if(cam.getTypeTDataPacket(&type_t) != CMUCAM4_STREAM_END)
    {
      E(CMUCAM4_RETURN_FAILURE);
    }

    if(cam.getTypeTDataPacket(&type_t) != CMUCAM4_SERIAL_TIMEOUT)
    {
      E(CMUCAM4_RETURN_FAILURE);
    }
  }

  E(cam.pollMode(false));

  E(cam.trackColor());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeTDataPacket(&type_t));
  }

/* Line Mode Test *************************************************************/
  Serial.print(F("\r\nTesting Line Mode... *******************"));

  E(cam.lineMode(true));

  E(cam.trackColor());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeTDataPacket(&type_t));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H1_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_1));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H2_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_2));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H4_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_4));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H8_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_8));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H16_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_16));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H32_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_32));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getHistogram(CMUCAM4_GREEN_CHANNEL, CMUCAM4_H64_BINS));

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeHDataPacket(&type_h_64));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.getMean());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeSDataPacket(&type_s));
    E(cam.getTypeFDataPacket(&type_f));
  }

  E(cam.lineMode(false));

/* Switching Mode Test ********************************************************/
  Serial.print(F("\r\nTesting Switching Mode... **************"));

  E(cam.switchingMode(true));
  E(cam.trackColor());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeTDataPacket(&type_t));
    E(cam.getTypeSDataPacket(&type_s));
  }

  E(cam.getMean());

  for(counter = 0; counter < 30; counter++)
  {
    E(cam.getTypeSDataPacket(&type_s));
    E(cam.getTypeTDataPacket(&type_t));
  }

  E(cam.switchingMode(false));

/* Histogram Tracking Test ****************************************************/
  Serial.print(F("\r\nTesting Histogram Tracking... **********"));

  E(cam.testMode(true));
  E(cam.setTrackingParameters(230, 255, 230, 255, 230, 255));

  E(cam.getMean());
  E(cam.getTypeSDataPacket(&type_s));

  if(((230 <= type_s.RMean) && (type_s.RMean <= 255)) &&
  ((230 <= type_s.GMean) && (type_s.GMean <= 255)) &&
  ((230 <= type_s.BMean) && (type_s.BMean <= 255)))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.histogramTracking(true));

  E(cam.getMean());
  E(cam.getTypeSDataPacket(&type_s));

  if((type_s.RMean < 230) || (255 < type_s.RMean) ||
  (type_s.GMean < 230) || (255 < type_s.GMean) ||
  (type_s.BMean < 230) || (255 < type_s.BMean))
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.histogramTracking(false));

  E(cam.testMode(true));
  E(cam.setTrackingParameters());

/* Inverted Filter Test *******************************************************/
  Serial.print(F("\r\nTesting Inverted Filter... *************"));

  E(cam.setTrackingParameters());
  E(cam.setTrackingWindow());

  E(cam.trackColor());
  E(cam.getTypeTDataPacket(&type_t));

  if(type_t.pixels != 255)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.invertedFilter(true));

  E(cam.trackColor());
  E(cam.getTypeTDataPacket(&type_t));

  if(type_t.pixels != 0)
  {
    E(CMUCAM4_RETURN_FAILURE);
  }

  E(cam.invertedFilter(false));

/* Noise Filter Test **********************************************************/
  Serial.print(F("\r\nTesting Noise Filter... ****************"));

  for(counter = 0; counter <= 160; counter++)
  {
    E(cam.noiseFilter(counter));

    E(cam.trackColor());
    E(cam.getTypeTDataPacket(&type_t));

    buffer = ((int) ceil(255.0 - (((float) counter) * (255.0 / 160.0))));

    if(type_t.pixels != buffer)
    {
      E(CMUCAM4_RETURN_FAILURE);
    }
  }

  E(cam.noiseFilter(0));

/* End Test *******************************************************************/
  Serial.print(F("\r\nTesting End... *************************"));

  E(cam.end());

/******************************************************************************/

  Serial.print(F("\r\n****************************************"));
  Serial.print(F("\r\nSoftware Acceptance Test Finalizing....."));
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
