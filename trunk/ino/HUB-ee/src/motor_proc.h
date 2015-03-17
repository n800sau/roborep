extern volatile int motor1QeiCounts, motor2QeiCounts;

void setup_motors();
void process_motors();

void mv_forward(int ms);
void mv_back(int ms);
void turn_left(int ms);
void turn_right(int ms);
