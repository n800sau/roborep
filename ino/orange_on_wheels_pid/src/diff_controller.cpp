/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

#include "orange_on_wheels_pid.h"
#include <math.h>

/* PID setpoint info For a Motor */

SetPointInfo leftPID, rightPID;

/* PID Parameters */
// proportional
int Kp = 20;
// derivative
int Kd = 12;
// integral
int Ki = 0;
//int Ko = 50;
int Ko = 40;

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p)
{
	long Perror;
	long output;
	int input;

	//Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
	input = p->Encoder - p->PrevEnc;
	Perror = p->TargetTicksPerFrame - input;


	/*
	* Avoid derivative kick and allow tuning changes,
	* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
	* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
	*/
	//output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
	// p->PrevErr = Perror;
	output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
	p->PrevEnc = p->Encoder;

	output += p->output;
	// Accumulate Integral error *or* Limit output.
	// Stop accumulating when output saturates
	if (output >= 100)
	  output = 100;
	else if (output <= -100)
	  output = -100;
	else
	/*
	* allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
	*/
	p->ITerm += Ki * Perror;

	p->output = output;
	p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
	/* Read the encoders */
	leftPID.Encoder = lCounter;
	rightPID.Encoder = rCounter;

//		nh.loginfo(("Left counter:" + String(leftPID.Encoder)).c_str());
//		nh.loginfo(("Right counter:" + String(rightPID.Encoder)).c_str());

	/* If we're not moving there is nothing more to do */
	if (full_stopped) {
		/*
		* Reset PIDs once, to prevent startup spikes,
		* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
		* PrevInput is considered a good proxy to detect
		* whether reset has already happened
		*/
		if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
	} else {
		/* Compute PID update for each motor */
		doPID(&rightPID);
		doPID(&leftPID);

//		nh.loginfo(("L PID:" + String(leftPID.output)).c_str());
//		nh.loginfo(("R PID:" + String(rightPID.output)).c_str());

		/* Set the motor speeds accordingly */
		setLeftMotor(leftPID.output);
		setRightMotor(rightPID.output);
	}
}

void resetPID()
{
	leftPID.TargetTicksPerFrame = 0.0;
	leftPID.Encoder = lCounter;
	leftPID.PrevEnc = leftPID.Encoder;
	leftPID.output = 0;
	leftPID.PrevInput = 0;
	leftPID.ITerm = 0;

	rightPID.TargetTicksPerFrame = 0.0;
	rightPID.Encoder = rCounter;
	rightPID.PrevEnc = rightPID.Encoder;
	rightPID.output = 0;
	rightPID.PrevInput = 0;
	rightPID.ITerm = 0;
}
