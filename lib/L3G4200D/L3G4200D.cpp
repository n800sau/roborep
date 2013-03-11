#include "L3G4200D.h"
#include <math.h>
#include <memory.h>
#include <syslog.h>
#include <unistd.h>

struct L3G4200D_CMD_FUNC:public CMD_FUNC {
	public:
		L3G4200D_CMD_FUNC(const char *cmd, L3G4200D::tFunction ptr) {
			this->cmd = cmd;
			this->ptr = ptr;
		}
		L3G4200D::tFunction ptr;
};

typedef L3G4200D_CMD_FUNC *pL3G4200D_CMD_FUNC;


static void L3G4200D_onADXL345(redisAsyncContext *c, void *reply, void *arg)
{
	L3G4200D *ths = (L3G4200D *)arg;
	redisReply *r = (redisReply *)reply;
	if (reply != NULL) {
		if (r->type == REDIS_REPLY_ARRAY) {
			for (int j = 0; j < r->elements; j++) {
				syslog(LOG_NOTICE, "%u) %s\n", j, r->element[j]->str);
			}
		}
	}
}

L3G4200D::L3G4200D(uint8_t address):ReServant("l3g4200d"),curr_g(),stop_g()
{
	for(int i=0; i< sizeof(zeroValue)/sizeof(zeroValue[0]); i++) {
		zeroValue[i] = 0;
	}
	m_Address = address;
	const static pL3G4200D_CMD_FUNC cmdlist[] = {
		new L3G4200D_CMD_FUNC("stop_state", &L3G4200D::stop_state),
	};
	this->setCmdList((pCMD_FUNC *)cmdlist, sizeof(cmdlist)/sizeof(cmdlist[0]));
}

void L3G4200D::call_cmd(const pCMD_FUNC cmd, json_t *js)
{
	pL3G4200D_CMD_FUNC ccmd = (pL3G4200D_CMD_FUNC)cmd;
	syslog(LOG_NOTICE, "Executing %s", ccmd->cmd);
	tFunction FunctionPointer = ccmd->ptr;
	(this->*FunctionPointer)(js);
	syslog(LOG_NOTICE, "%s finished", ccmd->cmd);
}

void L3G4200D::stop_state(json_t *js)
{
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	syslog(LOG_DEBUG, "stop state requested");
	
	// Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
	vector g;
	int count = 0;
	for (int i = 0; i < 100; i++) { // Take the average of 100 readings
		if(read(g)) {
			stop_g += g;
			count++;
			usleep(10000);
		}
	}
	stop_g /= count;
	stop_g.timestamp = dtime();
	const char *reply_key = json_string_value(json_object_get(js, "reply_key"));
	if(reply_key) {
		syslog(LOG_NOTICE, "replied to %s", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s OK", reply_key);
		redisAsyncCommand(aredis, NULL, NULL, "EXPIRE %s 30", reply_key);
	} else {
		syslog(LOG_NOTICE, "no reply_key");
	}
}


// Turns on the L3G4200D's gyro and places it in normal mode.
void L3G4200D::enableDefault(int scale)
{
	i2cwire.selectDevice(m_Address, "L3G4200D");
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
	i2cwire.writeToDevice(L3G4200D_CTRL_REG1, 0x0F);

  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  i2cwire.writeToDevice(L3G4200D_CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  i2cwire.writeToDevice(L3G4200D_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  i2cwire.writeToDevice(L3G4200D_CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    i2cwire.writeToDevice(L3G4200D_CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    i2cwire.writeToDevice(L3G4200D_CTRL_REG4, 0b00010000);
  }else{
    i2cwire.writeToDevice(L3G4200D_CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  i2cwire.writeToDevice(L3G4200D_CTRL_REG5, 0b00000000);

}

// Reads the 3 gyro channels and stores them in vector g
bool L3G4200D::read(vector &g)
{
	int16_t buf[3];
	bool rs = false;

	// assert the MSB of the address to get the gyro 
	// to do slave-transmit subaddress updating.
	i2cwire.selectDevice(m_Address, "L3G4200D");
	rs = i2cwire.requestFromDevice(L3G4200D_OUT_X_L | (1 << 7), 6, (uint8_t*)buf) == 6;

	g.x = buf[0];
	g.y = buf[1];
	g.z = buf[2];
	g.timestamp = dtime();

	return rs;
}

void L3G4200D::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G4200D::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G4200D::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

bool L3G4200D::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		enableDefault();
	// subscribe is buggy
	//	redisAsyncCommand(aredis, L3G4200D_onADXL345, NULL, "SUBSCRIBE adxl345");
	}
	return rs;
}

void L3G4200D::fill_json(json_t *js)
{
	json_t *sjs;

	sjs = json_object();
	json_object_set_new(sjs, "x", json_real(curr_g.x));
	json_object_set_new(sjs, "y", json_real(curr_g.y));
	json_object_set_new(sjs, "z", json_real(curr_g.z));
	json_object_set_new(sjs, "timestamp", json_real(curr_g.timestamp));
	json_object_set_new(js, "curr_g", sjs);

	sjs = json_object();
	json_object_set_new(sjs, "x", json_real(stop_g.x));
	json_object_set_new(sjs, "y", json_real(stop_g.y));
	json_object_set_new(sjs, "z", json_real(stop_g.z));
	json_object_set_new(sjs, "timestamp", json_real(stop_g.timestamp));
	json_object_set_new(js, "stop_g", sjs);
}

void L3G4200D::loop()
{
	if(read(curr_g)) {
		json2redislist();
	}

//	syslog(LOG_NOTICE, "gyro:%g %g %g\n", curr_g.x, curr_g.y, curr_g.z);

	ReServant::loop();
}
