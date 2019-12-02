#include <Adafruit_HMC5883_U.h>

extern sensors_event_t hmc5883_event;
extern float headingDegrees;
extern float compass_x;
extern float compass_y;
extern float compass_z;

void setup_compass();
void process_compass();
