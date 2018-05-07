#include "Wire.h"
#include "bin.h"

#define BINDATA CCS811_FW_cAppV1_1_0_XX000000_vLatest__bin

//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address

#define RESET_PIN  PA12
#define WAKE_PIN  PA11

#define CCS811_REG_STATUS         0x00
#define CCS811_HW_VERSION         0x21
#define CCS811_FW_BOOT_VERSION    0x23
#define CCS811_FW_APP_VERSION     0x24
#define CCS811_ERASE              0xF1
#define CCS811_REG_APP            0xF2
#define CCS811_VERIFY             0xF3

const uint8_t CCS811_ERASE_SEQ[] = { 0xE7, 0xA7, 0xE6, 0x09 };

typedef enum
{
	SENSOR_SUCCESS,
	SENSOR_ID_ERROR,
	SENSOR_I2C_ERROR,
	SENSOR_INTERNAL_ERROR,
	SENSOR_GENERIC_ERROR
	//...
} status;

void reset()
{
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  digitalWrite(WAKE_PIN, LOW);
  delay(100);
}

status writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	status rs = SENSOR_SUCCESS;

	Wire.beginTransmission(CCS811_ADDR);
	Wire.write(offset);
	Wire.write(dataToWrite);
	if( Wire.endTransmission() != 0 )
	{
		rs = SENSOR_I2C_ERROR;
	}
	printStatus(offset, rs);
	return rs;
}

status multiWriteRegister(uint8_t offset, const uint8_t *inputPointer, uint8_t length)
{
	status rs = SENSOR_SUCCESS;
	//define pointer that will point to the external space
	uint8_t i = 0;
	//Set the address
	Wire.beginTransmission(CCS811_ADDR);
	Wire.write(offset);
	while ( i < length )  // send data bytes
	{
		Wire.write(*inputPointer); // receive a byte as character
		inputPointer++;
		i++;
	}
	if( Wire.endTransmission() != 0 )
	{
		rs = SENSOR_I2C_ERROR;
	}
	printStatus(offset, rs);
	return rs;
}

status readRegister(uint8_t offset, uint8_t* outputPointer)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;
	status rs = SENSOR_SUCCESS;
	Wire.beginTransmission(CCS811_ADDR);
	Wire.write(offset);
	if( Wire.endTransmission() != 0 )
	{
		rs = SENSOR_I2C_ERROR;
	}
	Wire.requestFrom(CCS811_ADDR, numBytes);
	while ( Wire.available() ) // slave may send less than requested
	{
		result = Wire.read(); // receive a byte as a proper uint8_t
	}
	*outputPointer = result;

	return rs;
}

status multiReadRegister(uint8_t offset, uint8_t *outputPointer, uint8_t length)
{
	status rs = SENSOR_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	//Set the address
	Wire.beginTransmission(CCS811_ADDR);
	Wire.write(offset);
	if( Wire.endTransmission() != 0 )
	{
		rs = SENSOR_I2C_ERROR;
	}
	else  //OK, all worked, keep going
	{
		// request 6 bytes from slave device
		Wire.requestFrom(CCS811_ADDR, length);
		while ( (Wire.available()) && (i < length))  // slave may send less than requested
		{
			c = Wire.read(); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		//dump extra
		while(Wire.available())
		{
			Wire.read();
		}
	}


	return rs;
}

void program()
{
	reset();
	multiWriteRegister(CCS811_ERASE, CCS811_ERASE_SEQ, 4);
	delay(500);
	unsigned char *p;
	for(int i=0; i<sizeof(BINDATA); i+=8) {
		Serial.println(i);
		p = BINDATA + i;
		multiWriteRegister(CCS811_REG_APP, p, 8);
		delay(50);
	}
	delay(500);
	multiWriteRegister(CCS811_VERIFY, NULL, 0);
	delay(500);
	uint8_t st[61];
	readRegister(CCS811_REG_STATUS, st);
	if ((st[0] & 0x30) == 0x30) {
		Serial.println("Code valid. Success.");
	} else {
		Serial.println("Code invalid");
	}
}

void setup()
{
  Serial.begin(115200);
  Serial.println("CCS811 Programming");
delay(10000);
  Serial.print("BINDATA size:");
  Serial.println(sizeof(BINDATA));
  Serial.print("BINDATA remaining:");
  Serial.println(sizeof(BINDATA)%8);
//return;

  Wire.begin();

  pinMode(RESET_PIN, OUTPUT);   // set RESET pin as OUTPUT
  pinMode(WAKE_PIN, OUTPUT);   // set WAKE pin as OUTPUT

  program();

  reset();


    // initialize CCS811 and check version and status
  byte HWVersion;
  readRegister(CCS811_HW_VERSION, &HWVersion);
  Serial.print("CCS811 Hardware Version = 0x"); Serial.println(HWVersion, HEX);

  uint8_t FWBootVersion[2] = {0, 0}, FWAppVersion[2] = {0,0};
  multiReadRegister(CCS811_FW_BOOT_VERSION, FWBootVersion, 2);
  Serial.println("CCS811 Firmware Boot Version: ");
  Serial.print("Major = "); Serial.println((FWBootVersion[0] & 0xF0) >> 4);
  Serial.print("Minor = "); Serial.println(FWBootVersion[0] & 0x04);
  Serial.print("Trivial = "); Serial.println(FWBootVersion[1]);

  multiReadRegister(CCS811_FW_APP_VERSION, FWAppVersion, 2);
  Serial.println("CCS811 Firmware App Version: ");
  Serial.print("Major = "); Serial.println((FWAppVersion[0] & 0xF0) >> 4);
  Serial.print("Minor = "); Serial.println(FWAppVersion[0] & 0x04);
  Serial.print("Trivial = "); Serial.println(FWAppVersion[1]);


}

void printStatus(int cmd, status errorCode )
{
  Serial.print(cmd, HEX);
  Serial.print(":");
  switch ( errorCode )
  {
    case SENSOR_SUCCESS:
      Serial.println("SUCCESS");
      break;
    case SENSOR_ID_ERROR:
      Serial.println("ID_ERROR");
      break;
    case SENSOR_I2C_ERROR:
      Serial.println("I2C_ERROR");
      break;
    case SENSOR_INTERNAL_ERROR:
      Serial.println("INTERNAL_ERROR");
      break;
    case SENSOR_GENERIC_ERROR:
      Serial.println("GENERIC_ERROR");
      break;
    default:
      Serial.println("Unspecified error.");
  }
}

void loop()
{
	delay(1000);
}
