#include "raspiCamServant.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <aruco/cvdrawingutils.h>
#include <unistd.h>
#include <math.h>
#include <syslog.h>

//#define CAMERA_WIDTH 2592
//#define CAMERA_HEIGHT 1944

//#define CAMERA_WIDTH 1920
//#define CAMERA_HEIGHT 1080

#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 960

#define SUB_WIDTH 640
#define SUB_HEIGHT 480

//#define CAMERA_WIDTH 640
//#define CAMERA_HEIGHT 480

struct RASPICAM_CMD_FUNC:public CMD_FUNC {
	public:
		RASPICAM_CMD_FUNC(const char *cmd, raspiCamServant::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		raspiCamServant::tFunction ptr;
};

typedef RASPICAM_CMD_FUNC *pRASPICAM_CMD_FUNC;

raspiCamServant::raspiCamServant(const char *cam_yml):
ReServant("raspiCamServant"),markerSize(0.03),cam_yml(cam_yml),reply_type(RT_NONE)
{
	const static pRASPICAM_CMD_FUNC cmdlist[] = {
		new RASPICAM_CMD_FUNC("find_markers", &raspiCamServant::find_markers),
		new RASPICAM_CMD_FUNC("start_camera", &raspiCamServant::start_camera),
		new RASPICAM_CMD_FUNC("stop_camera", &raspiCamServant::stop_camera),
		new RASPICAM_CMD_FUNC("make_shot", &raspiCamServant::make_shot),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

bool raspiCamServant::create_servant()
{
	return ReServant::create_servant();
}

void raspiCamServant::destroy_servant()
{
	stop_camera();
	ReServant::destroy_servant();
}

void raspiCamServant::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pRASPICAM_CMD_FUNC ccmd = (pRASPICAM_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}


bool raspiCamServant::fill_json(json_t *js, int list_id)
{
	bool rs = false;
	switch(reply_type) {
		case RT_MARKERS:
			{
				syslog(LOG_NOTICE, "has %d markers", markers.size());
				json_t *mjsl = json_array();
				for(std::vector<aruco::Marker>::iterator m = markers.begin(); m != markers.end(); m++) {
					json_t *mjs = json_object();
					json_t *rjsl = json_array();
					for (int i=0; i<4; i++) {
						json_t *rjs = json_object();
						json_object_set_new(rjs, "x", json_real((*m)[i].x));
						json_object_set_new(rjs, "y", json_real((*m)[i].y));
						json_array_append(rjsl, rjs);
					}
					json_object_set_new(mjs, "coords", rjsl);
					json_object_set_new(mjs, "id", json_integer((*m).id));
					json_object_set_new(mjs, "size", json_real((*m).ssize));
					json_object_set_new(mjs, "width", json_integer(CAMERA_WIDTH));
					json_object_set_new(mjs, "height", json_integer(CAMERA_HEIGHT));
					json_array_append(mjsl, mjs);
				}
				json_object_set_new(js, "markers", mjsl);
			}
			rs = true;
			break;
		case RT_SHOT:
			rs = true;
			break;
	}
	return rs;
}

void raspiCamServant::loop()
{
	ReServant::loop();
}

void raspiCamServant::start_camera(json_t *js)
{
	stop_camera();
	//set camera params
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	camera.set(CV_CAP_PROP_BRIGHTNESS, 60);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
	if(cam_yml) {
		syslog(LOG_NOTICE, "Use %s", cam_yml);
		camParam.readFromXMLFile(cam_yml);
		//resizes the parameters to fit the size of the input image
		camParam.resize(cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
	}
	if (!camera.open()) {
		syslog(LOG_ERR, "Error opening the camera");
	} else {
		sleep(2);
		syslog(LOG_NOTICE, "Camera started");
	}
}

void raspiCamServant::stop_camera(json_t *js)
{
	if(camera.isOpened()) {
		camera.release();
		syslog(LOG_NOTICE, "Camera stopped");
	}
}

void raspiCamServant::make_shot(json_t *js)
{
	if(!camera.isOpened()) {
		start_camera();
	}
	const char *imgpath = json_string_value(json_object_get(js, "path"));
	cv::Mat image;
	reply_type = RT_SHOT;
	try
	{
		camera.grab();
		camera.retrieve(image);
		syslog(LOG_NOTICE, "captured dim: %dx%d", image.cols, image.rows);
		cv::imwrite(imgpath, image);
		// push image path to redis queue
		json2redislist();
	} catch (std::exception &ex) {
		reply_type = RT_NONE;
		syslog(LOG_ERR, "Exception : %s", ex.what());
	}
}

void raspiCamServant::find_markers(json_t *js)
{
	if(!camera.isOpened()) {
		start_camera();
	}
	const char *imgpath = json_string_value(json_object_get(js, "path"));
	bool draw_markers = json_is_true(json_object_get(js, "draw_markers"));
	cv::Mat image;
	markers.clear();
	reply_type = RT_MARKERS;
	try
	{

		camera.grab();
		camera.retrieve(image);
		cv::cvtColor(image, image, CV_BGR2GRAY);

		syslog(LOG_NOTICE, "captured dim: %dx%d", image.cols, image.rows);

		aruco::MarkerDetector mDetector;
		//read the input image
		//Ok, let's detect
		mDetector.detect(image, markers, camParam, markerSize);

		if(imgpath) {
			if(draw_markers) {
				cv::cvtColor(image, image, CV_GRAY2BGR);
				//for each marker, draw info and its boundaries in the image
				for (unsigned int i=0; i<markers.size(); i++) {
					markers[i].draw(image, cv::Scalar(0,0,255), 2);
				}
			}
			cv::imwrite(imgpath, image);
		}
		// push markers to redis queue
		json2redislist();
	} catch (std::exception &ex) {
		reply_type = RT_NONE;
		syslog(LOG_ERR, "Exception : %s", ex.what());
	}
}
