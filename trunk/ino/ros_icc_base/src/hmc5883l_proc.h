#include <Adafruit_HMC5883_U.h>

extern sensors_event_t hmc5883_event;
extern float headingDegrees;

void setup_compass();
void process_compass();
