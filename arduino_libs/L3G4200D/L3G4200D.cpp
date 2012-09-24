#include <L3G4200D.h>
#include <Wire.h>
#include <math.h>

L3G4200D::L3G4200D()
{
  m_Address = DefaultL3G4200D_Address;
}

L3G4200D::L3G4200D(uint8_t customAddress)
{
	m_Address = customAddress;
}


// Turns on the L3G4200D's gyro and places it in normal mode.
void L3G4200D::enableDefault(int scale)
{
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
  writeRegister(L3G4200D_CTRL_REG1, 0x0F);

  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_CTRL_REG5, 0b00000000);

}

// Reads the 3 gyro channels and stores them in vector g
int L3G4200D::read()
{
	int rs = 0;
	Wire.beginTransmission(m_Address);
	// assert the MSB of the address to get the gyro 
	// to do slave-transmit subaddress updating.
	Wire.write(L3G4200D_OUT_X_L | (1 << 7)); 
	Wire.endTransmission();
	Wire.requestFrom(m_Address, 6);

	if (Wire.available() == 6)
	{
	
		uint8_t xla = Wire.read();
		uint8_t xha = Wire.read();
		uint8_t yla = Wire.read();
		uint8_t yha = Wire.read();
		uint8_t zla = Wire.read();
		uint8_t zha = Wire.read();

		g.x = xha << 8 | xla;
		g.y = yha << 8 | yla;
		g.z = zha << 8 | zla;
		rs = 1;
	}
	Wire.endTransmission();
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

//from other source
void L3G4200D::writeRegister(byte address, byte val) {
    Wire.beginTransmission(m_Address); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int L3G4200D::readRegister(byte address){

    int v = -1;
    Wire.beginTransmission(m_Address);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(m_Address, 1); // read a byte

	int count=0;
    while(!Wire.available()) {
        // waiting
		delay(1);
		if(++count > 50)
			break;
    }

	if (Wire.available()) {
	    v = Wire.read();
	}
    return v;
}
