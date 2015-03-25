#include <Adafruit_Sensor.h>

typedef struct _adxl345_state {
	sensors_event_t event;
	char single_tap;
	inline void reset() { single_tap = 0; }
} adxl345_state_t;

extern adxl345_state_t adxl345_state;

void setup_accel();

void process_accel();

float acc_x_avg();
float acc_x_max();
