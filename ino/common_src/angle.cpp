#include <math.h>
#include "angle.h"

// angle difference in degrees
int angle_offset(int target_angle, int curr_angle)
{
	// make both in range 0-360
	int rs = (curr_angle - target_angle + 180) % 360;
	if (rs < 0)
		rs += 360;
	return rs - 180;
}
