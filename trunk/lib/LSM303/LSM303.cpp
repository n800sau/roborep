#include "LSM303.h"
#include <unistd.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define MAG_ADDRESS            (0x3C >> 1)
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(void):ReServant("lsm303")
{
  // These are just some values for a particular unit; it is recommended that
  // a calibration be done for your particular unit.
  m_max.x = +540; m_max.y = +500; m_max.z = 180;
  m_min.x = -520; m_min.y = -570; m_min.z = -770;
  
  _device = LSM303_DEVICE_AUTO;
  acc_address = ACC_ADDRESS_SA0_A_LOW;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

void LSM303::create_servant()
{
	ReServant::create_servant();
	init();
	enableDefault();
	// Calibration values. Use the Calibrate example program to get the values for
	// your compass.
	m_min.x = -520; m_min.y = -570; m_min.z = -770;
	m_max.x = +540; m_max.y = +500; m_max.z = 180;
}

// Public Methods //////////////////////////////////////////////////////////////

bool LSM303::timeoutOccurred()
{
  return did_timeout;
}

void LSM303::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int LSM303::getTimeout()
{
  return io_timeout;
}

void LSM303::init(uint8_t device, uint8_t sa0_a)
{  
  _device = device;
  switch (_device)
  {
    case LSM303DLH_DEVICE:
    case LSM303DLM_DEVICE:
      if (sa0_a == LSM303_SA0_A_LOW)
        acc_address = ACC_ADDRESS_SA0_A_LOW;
      else if (sa0_a == LSM303_SA0_A_HIGH)
        acc_address = ACC_ADDRESS_SA0_A_HIGH;
      else
        acc_address = (detectSA0_A() == LSM303_SA0_A_HIGH) ? ACC_ADDRESS_SA0_A_HIGH : ACC_ADDRESS_SA0_A_LOW;
      break;  
    
    case LSM303DLHC_DEVICE:
      acc_address = ACC_ADDRESS_SA0_A_HIGH;
      break;
      
    default:
      // try to auto-detect device
      if (detectSA0_A() == LSM303_SA0_A_HIGH)
      {
        // if device responds on 0011001b (SA0_A is high), assume DLHC
        acc_address = ACC_ADDRESS_SA0_A_HIGH;
        _device = LSM303DLHC_DEVICE;
      }
      else
      {
        // otherwise, assume DLH or DLM (pulled low by default on Pololu boards); query magnetometer WHO_AM_I to differentiate these two
        acc_address = ACC_ADDRESS_SA0_A_LOW;
        _device = (readMagReg(LSM303_WHO_AM_I_M) == 0x3C) ? LSM303DLM_DEVICE : LSM303DLH_DEVICE;
      }
  }
}

// Turns on the LSM303's accelerometer and magnetometers and places them in normal
// mode.
void LSM303::enableDefault(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303_CTRL_REG1_A, 0x27);
  
  // Enable Magnetometer
  // 0x00 = 0b00000000
  // Continuous conversion mode
  writeMagReg(LSM303_MR_REG_M, 0x00);
}

// Writes an accelerometer register
void LSM303::writeAccReg(uint8_t reg, uint8_t value)
{
	i2cwire.selectDevice(acc_address, myid());
	i2cwire.writeToDevice(reg, value);
}

// Reads an accelerometer register
uint8_t LSM303::readAccReg(uint8_t reg)
{
	uint8_t rs;
	i2cwire.selectDevice(acc_address, myid());
	i2cwire.requestFromDevice(reg, 1, &rs);
	return rs;
}

// Writes a magnetometer register
void LSM303::writeMagReg(uint8_t reg, uint8_t value)
{
	i2cwire.selectDevice(MAG_ADDRESS, myid());
	i2cwire.writeToDevice(reg, value);
}

// Reads a magnetometer register
uint8_t LSM303::readMagReg(int reg)
{
	uint8_t rs;
	// if dummy register address (magnetometer Y/Z), use device type to determine actual address
	if (reg < 0)
	{
		switch (reg)
		{
		case LSM303_OUT_Y_H_M:
			reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_H_M : LSM303DLM_OUT_Y_H_M;
			break;
		case LSM303_OUT_Y_L_M:
			reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_L_M : LSM303DLM_OUT_Y_L_M;
			break;
		case LSM303_OUT_Z_H_M:
			reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_H_M : LSM303DLM_OUT_Z_H_M;
			break;
		case LSM303_OUT_Z_L_M:
			reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_L_M : LSM303DLM_OUT_Z_L_M;
			break;
		}
	}
	i2cwire.selectDevice(MAG_ADDRESS, myid());
	i2cwire.requestFromDevice(reg, 1, &rs);
	return rs;
}

void LSM303::setMagGain(magGain value)
{
	i2cwire.selectDevice(MAG_ADDRESS, myid());
	i2cwire.writeToDevice(LSM303_CRB_REG_M, (uint8_t) value);
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void)
{
	uint16_t coords[3];
	i2cwire.selectDevice(acc_address, myid());
	i2cwire.requestFromDevice(LSM303_OUT_X_L_A | (1 << 7), 6, (uint8_t *)coords);
	// shift right to discard lowest 4 bits (which are meaningless)
	// GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
	// if you port it to a compiler that does a logical right shift instead.
	a.x = coords[0] >> 4;
	a.y = coords[1] >> 4;
	a.z = coords[2] >> 4;
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void)
{
	uint8_t coords[3];
	i2cwire.selectDevice(MAG_ADDRESS, myid());
	i2cwire.requestFromDevice(LSM303_OUT_X_H_M, 6, (uint8_t*)&coords);
	m.x = (coords[0] << 8) + (coords[1] >> 8);
	if (_device == LSM303DLH_DEVICE)
	{
		// DLH: register address for Y comes before Z
		m.y = (coords[2] << 8) + (coords[3] >> 8);
		m.z = (coords[4] << 8) + (coords[5] >> 8);
	} else {
		// DLM, DLHC: register address for Z comes before Y
		m.z = (coords[2] << 8) + (coords[3] >> 8);
		m.y = (coords[4] << 8) + (coords[5] >> 8);
	}
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
  readAcc();
  readMag();
}

// Returns the number of degrees from the -Y axis that it
// is pointing.
int LSM303::heading(void)
{
  return heading((vector){0,-1,0});
}

// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
// 
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int LSM303::heading(vector from)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    vector temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    vector E;
    vector N;
    vector_cross(&m, &temp_a, &E);
    vector_normalize(&E);
    vector_cross(&temp_a, &E, &N);
  
    // compute heading
    int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
  return heading;
}

void LSM303::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float LSM303::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void LSM303::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

uint8_t LSM303::detectSA0_A(void)
{
	uint8_t rs;
	i2cwire.selectDevice(ACC_ADDRESS_SA0_A_LOW, myid());
	return (i2cwire.requestFromDevice(LSM303_CTRL_REG1_A, 1, &rs)) ? LSM303_SA0_A_LOW : LSM303_SA0_A_HIGH;
}

void LSM303::fill_json(json_t *js)
{
	read();
	int heading = this->heading((LSM303::vector){0,-1,0});
	json_object_set_new(js, "heading", json_integer(heading));
}

void LSM303::loop()
{
	json2redislist();
	ReServant::loop();
}
