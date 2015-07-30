/*
extern bool compass_mode;

// left reverse
extern volatile bool lReverse;
// right reverse
extern volatile bool rReverse;

// count of encoder ticks until stop
extern int LstepSize;
extern int RstepSize;

// turn step size
extern int TstepSize;

extern int intentDir;
extern int azimuth ;
extern int azimuth_allowance ;
extern int offset;

extern volatile double fi, x, y;
*/

extern volatile int lDest;
extern volatile int lPower;

extern volatile int rDest;
extern volatile int rPower;

// encoder distance counter
extern volatile int lCounter;
extern volatile int rCounter;

// min and max power (0-255)
// default 255
extern int max_motor_power;
// default 200
extern int min_motor_power;

void mv_forward(int steps=2);
void mv_back(int steps=1);
void turn_left(int steps=1);
void turn_right(int steps=1);

void motor_setup();
void motor_process();

void stop(bool reset_azimuth=false);

