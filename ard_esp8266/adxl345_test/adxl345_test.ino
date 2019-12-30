/**
* This Arduino code implements some of the advanced features available in the 
* ADXL345 accelerometer such as power management, single tap, double tap, freefall
*/

#include <Wire.h>
#include "binary_const.h" // with this we can use something B8(01010101) that it will convert to 85 at compile time
// the above cames really handy and readable when doing per bit configuration in the ADXL345 registers

#define DEVICE (0x53)    //ADXL345 device address (with SDO tied to ground)
//#define DEVICE (0x1D)    //ADXL345 device address (with SDO tied to ground)
#define TO_READ (6)      //num of bytes we are going to read each time (two bytes for each axis)

#define INTERRUPTPIN 5   // Arduino pin which is connected to INT1 from the ADXL345


// Register map: see ADXL345 datasheet page 14
const int R_DEVID = 0;
const int R_THRESH_TAP = 29;
const int R_OFSX = 30;
const int R_OFSY = 31;
const int R_OFSZ = 32;
const int R_DUR = 33;
const int R_LATENT = 34;
const int R_WINDOW = 35;
const int R_THRESH_ACT = 36;
const int R_THRESH_INACT = 37;
const int R_TIME_INACT = 38;
const int R_ACT_INACT_CTL = 39;
const int R_THRESH_FF = 40;
const int R_TIME_FF = 41;
const int R_TAP_AXES = 42;
const int R_ACT_TAP_STATUS = 43;
const int R_BW_RATE = 44;
const int R_POWER_CTL = 45;
const int R_INT_ENABLE = 46;
const int R_INT_MAP = 47;
const int R_INT_SOURCE = 48;
const int R_DATA_FORMAT = 49;
const int R_DATAX0 = 50;
const int R_DATAX1 = 51;
const int R_DATAY0 = 52;
const int R_DATAY1 = 53;
const int R_DATAZ0 = 54;
const int R_DATAZ1 = 55;
const int R_FIFO_CTL = 56;
const int R_FIFO_STATUS = 57;


byte buff[TO_READ];    //6 bytes buffer for saving data read from the device
char str[512];                      //string buffer to transform data before sending it to the serial port
boolean inspected = 0;


void setup()
{
  Serial.begin(115200);  // start serial for output

	//sda scl
  Wire.pins(12, 14);
  Wire.begin();        // join i2c bus (address optional for master)
//	pinMode(ESP_PINS_OFFSET + 12, INPUT_PULLUP);
//	pinMode(ESP_PINS_OFFSET + 13, INPUT_PULLUP);

  
  pinMode(INTERRUPTPIN, INPUT); 

	// 10 bit
//  writeTo(DEVICE, R_DATA_FORMAT, 0); // 00000000

	// full res
  writeTo(DEVICE, R_DATA_FORMAT, 0x8); // 00001000
  
  
  writeTo(DEVICE, R_OFSX, 17); // offset for x
  writeTo(DEVICE, R_OFSY, 0); // offset for y
  writeTo(DEVICE, R_OFSZ, -0x7f); // offset for z

  // interrupts setup
  writeTo(DEVICE, R_INT_MAP, 0); // send all interrupts to ADXL345's INT1 pin
  writeTo(DEVICE, R_INT_ENABLE, B8(1111100)); // enable signle and double tap, activity, inactivity and free fall detection
  
  
  // free fall configuration
  writeTo(DEVICE, R_TIME_FF, 0x14); // set free fall time
  writeTo(DEVICE, R_THRESH_FF, 0x05); // set free fall threshold
  
  // single tap configuration
  writeTo(DEVICE, R_DUR, 0x1F); // 625us/LSB
  writeTo(DEVICE, R_THRESH_TAP, 24); // 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
//  writeTo(DEVICE, R_THRESH_TAP, 48); // 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
  writeTo(DEVICE, R_TAP_AXES, B8(111)); // enable tap detection on x,y,z axes

  // double tap configuration
  writeTo(DEVICE, R_LATENT, 0x10);
  writeTo(DEVICE, R_WINDOW, 0xFF);
  
  // inactivity configuration
  writeTo(DEVICE, R_TIME_INACT, 10); // 1s / LSB
  writeTo(DEVICE, R_THRESH_INACT, 3); // 62.5mg / LSB
  // also working good with high movements: R_TIME_INACT=5, R_THRESH_INACT=16, R_ACT_INACT_CTL=B8(00000111)
  // but unusable for a quite slow movements
  
  // activity configuration
  writeTo(DEVICE, R_THRESH_ACT, 8); // 62.5mg / LSB
  
  // activity and inctivity control
  writeTo(DEVICE, R_ACT_INACT_CTL, B8(11111111)); // enable activity and inactivity detection on x,y,z using ac
  
  // set the ADXL345 in measurement and sleep Mode: this will save power while while we will still be able to detect activity
  // set the Link bit to 1 so that the activity and inactivity functions aren't concurrent but alternatively activated
  // set the AUTO_SLEEP bit to 1 so that the device automatically goes to sleep when it detects inactivity
  writeTo(DEVICE, R_POWER_CTL, B8(111100));

  Serial.println("End of setup");
}

/*
Prints out the state of all registers
void inspectRegisters() {
  int regCount = R_FIFO_STATUS - R_THRESH_TAP + 1;
  byte regBuff[regCount];
  readFrom(DEVICE, R_THRESH_TAP, regCount, regBuff);
  for(int i=0; i<regCount; i++) {
    Serial.println(R_THRESH_TAP + i);
    Serial.println(regBuff[i], BIN);
  }
}
*/

void loop()
{
 
	/* 
  if(inspected == 0) {
    //delay(1000); 
    Serial.println("inspecting registers");
    inspectRegisters();
    delay(1000);
    inspected = 1;
  }
  */
  
  // we use a digitalRead instead of attachInterrupt so that we can use delay()
  if(digitalRead(INTERRUPTPIN)) {
    int interruptSource = readByte(DEVICE, R_INT_SOURCE);
    //Serial.print("### ");
    //Serial.println(interruptSource, BIN);
    
    
    if(interruptSource & B8(100)) {
      Serial.println("### FREE_FALL");
    }
    if(interruptSource & B8(1000)) {
      Serial.println("### Inactivity");
      // we don't need to put the device in sleep because we set the AUTO_SLEEP bit to 1 in R_POWER_CTL
      // set the LOW_POWER bit to 1 in R_BW_RATE: with this we get worst measurements but we save power
      int bwRate = readByte(DEVICE, R_BW_RATE);
      writeTo(DEVICE, R_BW_RATE, bwRate | B8(10000));
    }
    if(interruptSource & B8(10000)) {
      Serial.println("### Activity");
      
      // get current power mode
      int powerCTL = readByte(DEVICE, R_POWER_CTL);
      // set the device back in measurement mode
      // as suggested on the datasheet, we put it in standby then in measurement mode
      // we do this using a bitwise and (&) so that we keep the current R_POWER_CTL configuration
      writeTo(DEVICE, R_POWER_CTL, powerCTL & B8(11110011));
      delay(10); // let's give it some time (not sure if this is needed)
      writeTo(DEVICE, R_POWER_CTL, powerCTL & B8(11111011));
      
      // set the LOW_POWER bit to 0 in R_BW_RATE: get back to full accuracy measurement (we will consume more power)
      int bwRate = readByte(DEVICE, R_BW_RATE);
      writeTo(DEVICE, R_BW_RATE, bwRate & B8(01111));
    }
    if(interruptSource & B8(100000)) {
      Serial.print("### DOUBLE_TAP Axes: ");
      printTapAxes();
      Serial.println(""); // closing Axes line
    }
    else if(interruptSource & B8(1000000)) { // when a double tap is detected also a signle tap is deteced. we use an else here so that we only print the double tap
      Serial.print("### SINGLE_TAP Axes: ");
      printTapAxes();
      Serial.println(""); // closing Axes line
    }
    delay(5);
  }
  
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  short x, y, z;
  
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];

// 10 bit = 0x200
//#define RANGE_2G_MULTIPLIER (1./0x200)

// full res
#define RANGE_2G_MULTIPLIER (0.004)

#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */

	float fx, fy, fz;
//	fx = x * RANGE_2G_MULTIPLIER * SENSORS_GRAVITY_EARTH;
//	fy = y * RANGE_2G_MULTIPLIER * SENSORS_GRAVITY_EARTH;
//	fz = z * RANGE_2G_MULTIPLIER * SENSORS_GRAVITY_EARTH;

	fx = x * RANGE_2G_MULTIPLIER;
	fy = y * RANGE_2G_MULTIPLIER;
	fz = z * RANGE_2G_MULTIPLIER;

	for(int j=0; j<6; j++) {
		Serial.print(buff[j], HEX);
		Serial.print(" ");
	}
//	Serial.println("");

	Serial.print(fx);
	Serial.print(" ");
	Serial.print(fy);
	Serial.print(" ");
	Serial.println(fz);
  
  //we send the x y z values as a string to the serial port
//  sprintf(str, "%d %d %d", x, y, z);  
//  Serial.print(str);
//  Serial.println();
  
  //It appears that delay is needed in order not to clog the port
  delay(15);
}


void printTapAxes() {
  int tapStatus = readByte(DEVICE, R_ACT_TAP_STATUS);
  if(tapStatus & B8(100)) {
    Serial.print("x ");
  }
  if(tapStatus & B8(10)) {
    Serial.print("y ");
  }
  if(tapStatus & B8(1)) {
    Serial.print("z ");
  }
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}


// read a single bite and returns the readed value
byte readByte(int device, byte address) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, 1);    // request 1 byte from device
  
  int readed = 0;
  if(Wire.available())
  { 
    readed = Wire.read(); // receive a byte
  }
  Wire.endTransmission(); //end transmission
  return readed;
}
