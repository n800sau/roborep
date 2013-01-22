#include <unistd.h>
#include "CMUcam4.h"
#include "CMUcom4.h"
#include "wiringPi.h"

#define RED_TOLERANCE 30
#define GREEN_TOLERANCE 30
#define BLUE_TOLERANCE 30

#define LED_BLINK 2 // 2 Hz
#define LED_SETUP 5 // 5 Hz
#define POSE_TIME 5000 // 5 seconds
#define WAIT_TIME 5000 // 5 seconds

CMUcam4 cam(CMUCOM4_SERIAL);

void cam_begin()
{
	pinMode(17, OUTPUT);
	pinMode(24, OUTPUT);
	digitalWrite(17, false);
	digitalWrite(24, true);
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

  cam.LEDOn(LED_SETUP);
  usleep(WAIT_TIME*1000);

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
  usleep(WAIT_TIME*1000);

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

	usleep(POSE_TIME*1000);

  // So take a picture.
//  cam.dumpFrame(CMUCAM4_HR_160, CMUCAM4_VR_120);
//  cam.unmountDisk();
}

void start()
{
	setup();
	while(true) {
		loop();
	}
}

int main(int argc, const char **argv)
{
	wiringPiSetup();
	start();
	return 0;
}

