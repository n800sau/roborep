#ifndef KALMAN_h
#define KALMAN_h

#include <reservant.h>
#include "KalmanFilter.h"

class Kalman:public ReServant
{
	private:
		KalmanFilter kX;
		double timerX;
		double angleX;
		double rateX;
		double pitchX;

		void stop_state(json_t *js);

	protected:
		virtual void create_servant();
		virtual void loop();
		virtual void fill_json(json_t *js);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
		double getAngleX(json_t *acc_js, json_t *stop_acc_js);
		double getRateX(json_t *gyro_js, json_t *stop_gyro_js);

	public:
		double stop_time;

		Kalman();

		typedef void (Kalman::*tFunction)(json_t *js);
};

#endif



