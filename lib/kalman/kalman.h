#ifndef KALMAN_h
#define KALMAN_h

#include <reservant.h>
#include "KalmanFilter.h"

class Kalman:public ReServant
{
	private:
		struct KFilter {
			KalmanFilter k;
			double timer;
			double angle;
			double rate;
			double pitch;
			KFilter():timer(dtime()),angle(0),rate(0),pitch(0),k() {}
		} x,y,z;

		void stop_state(json_t *js);

	protected:
		virtual bool create_servant();
		virtual void loop();
		virtual bool fill_json(json_t *js, int list_id);
		virtual void call_cmd(const pCMD_FUNC cmd, json_t *js);
		double getAngleX(json_t *acc_js, json_t *stop_acc_js, const char *X, const char *Z);
		double getRateX(json_t *gyro_js, json_t *stop_gyro_js, const char *X);

	public:
		double stop_time;

		Kalman();

		typedef void (Kalman::*tFunction)(json_t *js);
};

#endif



