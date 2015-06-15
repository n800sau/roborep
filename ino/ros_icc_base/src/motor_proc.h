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

extern volatile int lDest;
extern volatile int lPower;

extern volatile int rDest;
extern volatile int rPower;

extern int intentDir;
extern int azimuth ;
extern int azimuth_allowance ;
extern int offset;

extern volatile double fi, x, y;
*/

// encoder distance counter
extern volatile int lCounter;
extern volatile int rCounter;

void mv_forward(long ms);
void mv_back(long ms);
void turn_left(long ms);
void turn_right(long ms);

void motor_setup();
void motor_process();

void stop(bool reset_azimuth=false);

