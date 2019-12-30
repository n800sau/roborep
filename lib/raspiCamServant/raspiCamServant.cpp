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
ReServant("raspiCamServant"),markerSize(0.03),cam_yml(cam_yml),reply_type(RT_NONE),pwriter(NULL)
{
	const static pRASPICAM_CMD_FUNC cmdlist[] = {
		new RASPICAM_CMD_FUNC("find_markers", &raspiCamServant::find_markers),
		new RASPICAM_CMD_FUNC("start_camera", &raspiCamServant::start_camera),
		new RASPICAM_CMD_FUNC("stop_camera", &raspiCamServant::stop_camera),
		new RASPICAM_CMD_FUNC("make_shot", &raspiCamServant::make_shot),
		new RASPICAM_CMD_FUNC("start_video", &raspiCamServant::start_video),
		new RASPICAM_CMD_FUNC("stop_video", &raspiCamServant::stop_video),
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
					json_object_set_new(mjs, "width", json_integer(camera.get(CV_CAP_PROP_FRAME_WIDTH)));
					json_object_set_new(mjs, "height", json_integer(camera.get(CV_CAP_PROP_FRAME_HEIGHT)));
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
	if(pwriter) {
		cv::Mat image;
		camera.grab();
		camera.retrieve(image);
		pwriter->write(image);
	}
	ReServant::loop();
}

void raspiCamServant::start_camera(json_t *js)
{
	json_t *obj;
	stop_camera();
	//set camera params
	obj = json_object_get(js, "width");
	int w = (obj) ? json_integer_value(obj) :  CAMERA_WIDTH;
	obj = json_object_get(js, "height");
	int h = (obj) ? json_integer_value(obj) :  CAMERA_HEIGHT;
	camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	obj = json_object_get(js, "brightness");
	camera.set(CV_CAP_PROP_BRIGHTNESS, (obj) ? json_integer_value(obj) : 60);
	obj = json_object_get(js, "contrast");
	camera.set(CV_CAP_PROP_CONTRAST, (obj) ? json_integer_value(obj) : 60);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, w);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, h);
	if(cam_yml) {
		syslog(LOG_NOTICE, "Use %s", cam_yml);
		camParam.readFromXMLFile(cam_yml);
		//resizes the parameters to fit the size of the input image
		camParam.resize(cv::Size(w, h));
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
		start_camera(js);
	}
	const char *imgpath = json_string_value(json_object_get(js, "path"));
	cv::Mat image;
	reply_type = RT_SHOT;
	try
	{
		camera.grab();
		camera.retrieve(image);
		syslog(LOG_NOTICE, "captured dim: %dx%d", image.cols, image.rows);
		if(strncmp(imgpath, "redis:", 6) == 0) {
			vector<uchar> v;
			cv::imencode(".jpg", image, v);
			uchar *buf = new uchar[v.size()];
			std::copy(v.begin(), v.end(), buf);
			redisCommand(redis, "SET %s %b", (imgpath + 6), buf, v.size());
		} else {
			cv::imwrite(imgpath, image);
		}
		// push image path to redis queue
		json2redislist();
	} catch (std::exception &ex) {
		reply_type = RT_NONE;
		syslog(LOG_ERR, "Exception : %s", ex.what());
	}
}

void raspiCamServant::start_video(json_t *js)
{
	if(!camera.isOpened()) {
		start_camera(js);
	}
	const char *avipath = json_string_value(json_object_get(js, "path"));
	reply_type = RT_VIDEO;
	double fps = json_number_value(json_object_get(js, "fps"));
	if( fps == 0 ) fps = 25;
	setLoopInterval(1./fps);
	cv::Size sz = cv::Size(camera.get(CV_CAP_PROP_FRAME_WIDTH), camera.get(CV_CAP_PROP_FRAME_HEIGHT));
	pwriter = new cv::VideoWriter(avipath, CV_FOURCC('X','V','I','D'), fps, sz);
	if(!pwriter) {
		reply_type = RT_NONE;
		syslog(LOG_ERR, "Could not open the output video to write: %s", avipath);
	}
}

void raspiCamServant::stop_video(json_t *js)
{
	if(pwriter) {
		delete pwriter;
		pwriter = NULL;
		setLoopInterval();
	}
}

static bool marker_eq(aruco::Marker i, aruco::Marker j) {
  return i.id == j.id;
}

static bool marker_less(aruco::Marker i,aruco::Marker j) {
	return i.id < j.id;
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

		printf("Dim: %dx%d\n", image.cols, image.rows);

		syslog(LOG_NOTICE, "captured dim: %dx%d", image.cols, image.rows);

		aruco::MarkerDetector mDetector;

		//read the input image
		//Ok, let's detect
		mDetector.detect(image, markers, camParam, markerSize);

		if(image.cols > 320 && image.rows > 240) {
			int roi_w = 320;
			int roi_h = 240;
			const int x_step = 50;
			const int y_step = 50;
			for(int x=0; x<image.cols; x+=x_step) {
				for(int y=0; y<image.rows; y+=y_step) {
					cv::Mat subimage(image, cv::Rect(x, y, std::min(image.cols-x, roi_w), std::min(image.rows-y, roi_h)));
//					char *buf = (char*)malloc(strlen(imgpath) + 100);
//					snprintf(buf, strlen(imgpath) + 100, "%s.%dx%d.png", imgpath, x, y);
//					cv::imwrite(buf, subimage);
//					free(buf);
					printf("Sub Dim: (%d,%d) %dx%d\n", x, y, subimage.cols, subimage.rows);
					vector<aruco::Marker> submarkers;
					mDetector.detect(subimage, submarkers, camParam, markerSize);
					if(!submarkers.empty())
						printf("Detected %d\n", submarkers.size());
					while(!submarkers.empty()) {
						aruco::Marker m = submarkers.back();
						submarkers.pop_back();
						for(int j=0; j<4; j++) {
							m[j].x += x;
							m[j].y += y;
						}
						markers.push_back(m);
					}
				}
			}
		}
		std::sort(markers.begin(), markers.end(), marker_less);
		markers.erase(std::unique(markers.begin(), markers.end(), marker_eq), markers.end());

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
