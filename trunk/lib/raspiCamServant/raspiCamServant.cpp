#include "raspiCamServant.h"
#include <unistd.h>
#include <math.h>
#include <syslog.h>

struct RASPICAM_CMD_FUNC:public CMD_FUNC {
	public:
		RASPICAM_CMD_FUNC(const char *cmd, raspiCamServant::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		raspiCamServant::tFunction ptr;
};

typedef RASPICAM_CMD_FUNC *pRASPICAM_CMD_FUNC;

raspiCamServant::raspiCamServant(const char *cam_yml):ReServant("raspiCamServant"),markerSize(0.03),cam_yml(cam_yml)
{
	const static pRASPICAM_CMD_FUNC cmdlist[] = {
		new RASPICAM_CMD_FUNC("find_markers", &raspiCamServant::find_markers),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

bool raspiCamServant::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		if (!camera.open()) {
			syslog(LOG_ERR, "Error opening the camera");
		} else {
			//set camera params
			camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
			camera.set(CV_CAP_PROP_BRIGHTNESS, 70);
//			camera.set(CV_CAP_PROP_FRAME_WIDTH, 2592);
//			camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1944);
			camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
			camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
			sleep(4);
			if(cam_yml) {
				syslog(LOG_NOTICE, "Use %s", cam_yml);
				camParam.readFromXMLFile(cam_yml);
			}
		}
	}
	return rs;
}

void raspiCamServant::destroy_servant()
{
	camera.release();
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
	return false;
}

void raspiCamServant::loop()
{
	json2redislist();
	ReServant::loop();
}

void raspiCamServant::find_markers(json_t *js)
{
	syslog(LOG_DEBUG, "find markers requested");
}
