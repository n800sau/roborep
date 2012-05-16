#ifndef __CHASSIS_CMD__

#define __CHASSIS_CMD__

void chassisSetup();

//right
void motorRight(int pwm, boolean reverse = false);

//left
void motorLeft(int pwm, boolean reverse = false);

#endif //__CHASSIS_CMD__
