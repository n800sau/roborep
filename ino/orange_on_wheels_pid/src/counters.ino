volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;

void lIntCB()
{
	unsigned long t = micros();
	if( t - intLtime > threshold )
	{
		lVel = lDirection * ENC_STEP/t;
		intLtime = t;
		lCounter += lDirection;
	}
}

void rIntCB()
{
	unsigned long t = micros();
	if( t - intRtime > threshold )
	{
		rVel = rDirection * ENC_STEP/t;
		intRtime = t;
		rCounter += rDirection;
	}
}

