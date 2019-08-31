namespace motion {

	extern float left, right;
	extern double lSet, rSet; // PID Required speed for each wheel
	extern double lOut, rOut; // PID Output command for each wheel

	void setup();
	void tick();
	void stop();
	void set_motion(float x, float th);

}
