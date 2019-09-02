#include <Wire.h>
#include "vl53l0x_proc"

int vl53l0x_mm = 0;

VL53L0X vl53l0x_sensor;

void setup_tof()
{
  Wire.begin();

  vl53l0x_sensor.init();
  vl53l0x_sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  vl53l0x_sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  vl53l0x_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  vl53l0x_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  vl53l0x_sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  vl53l0x_sensor.setMeasurementTimingBudget(200000);
#endif

}

void process_tof()
{
	int mm = vl53l0x_sensor.readRangeSingleMillimeters();
	if (vl53l0x_sensor.timeoutOccurred()) {
		Serial.print(" TIMEOUT");
	} else {
		vl53l0x_mm = mm;
	}
}
