#ifndef __raspiCamServant_H

#define __raspiCamServant_H

#include <reservant.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam_still_cv.h>

class raspiCamServant:public ReServant
{
	private:
		raspicam::RaspiCam_Cv camera;
		aruco::CameraParameters camParam;
		float markerSize;
		const char *cam_yml;

	protected:
		virtual void loop();
		virtual bool fill_json(json_t *js, int list_id);
		void call_cmd(const pCMD_FUNC cmd, json_t *js);
		void find_markers(json_t *js);

	public:
		raspiCamServant(const char *cam_yml=NULL);
		virtual bool create_servant();
		virtual void destroy_servant();

		typedef void (raspiCamServant::*tFunction)(json_t *js);

};

#endif
