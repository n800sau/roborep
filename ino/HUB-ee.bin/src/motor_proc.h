extern volatile int motor1QeiCounts, motor2QeiCounts;
extern float motor1Coef, motor2Coef;

void setup_motors();
void process_motors();

void mv_forward(long ms);
void mv_back(long ms);
void turn_left(long ms);
void turn_right(long ms);

void stop();
bool stopped();

void calibrate_motors();

