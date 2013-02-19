#include "L3G4200D.h"
#include <math.h>
#include <memory.h>
#include <syslog.h>
#include "Kalman.h"

L3G4200D::L3G4200D(uint8_t address):ReServant("l3g4200d"),g()
{
	for(int i=0; i< sizeof(zeroValue)/sizeof(zeroValue[0]); i++) {
		zeroValue[i] = 0;
	}
	m_Address = address;
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
bool L3G4200D::read()
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

void L3G4200D::create_servant()
{
	ReServant::create_servant();
	enableDefault();
}

void L3G4200D::loop()
{
	if(read()) {
		json_t *js = json_object();
		json_object_set_new(js, "mypath", json_string(mypath()));
		json_object_set_new(js, "s_timestamp", json_string(s_timestamp()));
		json_object_set_new(js, "x", json_real(g.x));
		json_object_set_new(js, "y", json_real(g.y));
		json_object_set_new(js, "z", json_real(g.z));
		char *jstr = json_dumps(js, JSON_INDENT(4));
		if(jstr) {
			syslog(LOG_DEBUG, "%s\n", jstr);
			redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s.js.obj %s", myid(), jstr);
			redisAsyncCommand(aredis, NULL, NULL, "LTRIM %s.js.obj %d -1", myid(), -REDIS_LIST_SIZE-1);
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not encode JSON\n");
		}
		json_decref(js);


//		redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.x %g", myid(), g.x);
//		redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.y %g", myid(), g.y);
//		redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.z %g", myid(), g.z);
	}

	//kalman filter

	syslog(LOG_NOTICE, "gyro:%g %g %g\n", g.x, g.y, g.z);

	ReServant::loop();
}
