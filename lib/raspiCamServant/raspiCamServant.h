#ifndef __raspiCamServant_H

#define __raspiCamServant_H

#include <reservant.h>
#include <aruco/aruco.h>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam_still_cv.h>
#include "opencv2/video/video.hpp"

enum RTYPE { RT_NONE, RT_MARKERS, RT_SHOT, RT_VIDEO };

class raspiCamServant:public ReServant
{
	private:
		// video camera
		raspicam::RaspiCam_Cv camera;

		// still camera
//		raspicam::RaspiCamStill_Cv camera;

		aruco::CameraParameters camParam;
		float markerSize;
		const char *cam_yml;
		vector<aruco::Marker> markers;
		RTYPE reply_type;
		cv::VideoWriter *pwriter;

	protected:
		virtual void loop();
		virtual bool fill_json(json_t *js, int list_id);
		void call_cmd(const pCMD_FUNC cmd, json_t *js);
		void find_markers(json_t *js);
		void start_camera(json_t *js=NULL);
		void stop_camera(json_t *js=NULL);
		void make_shot(json_t *js=NULL);
		void start_video(json_t *js=NULL);
		void stop_video(json_t *js=NULL);

	public:
		raspiCamServant(const char *cam_yml=NULL);
		virtual bool create_servant();
		virtual void destroy_servant();

		typedef void (raspiCamServant::*tFunction)(json_t *js);

};

#endif
