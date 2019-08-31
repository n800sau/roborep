namespace encoders {
	extern volatile int lCounter;
	extern volatile int rCounter;
	extern volatile int lCounter;
	extern volatile int rCounter;
	extern volatile bool lFwd;
	extern volatile bool rFwd;
	void setup();
	void reset();
	double count2dist(int count);
}
