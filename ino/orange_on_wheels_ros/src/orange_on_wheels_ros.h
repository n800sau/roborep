#define MAX_STOP_DIST 0.3

void stop(bool full=false);
void setLeftMotor(int power, bool fwd);
void setRightMotor(int power, bool fwd);
void stop_after(int timeout);
void move2release(int pwr, bool fwd);
void straight(int pwr, bool fwd);
void resetCounters();
float getRange_HeadUltrasound(int attempts=2);
float getRange_BackUltrasound(int attempts=2);


