#ifndef __SERIAL_CMD__

#define __SERIAL_CMD__

void serialSetup(int i2c_addr);
int processCmdline(String &cmdline, String c_args[], int n);
int getIntVal(const String &strval);
int readSerial(String c_args[], int max_c_args);
int readI2C(String c_args[], int max_c_args);

#endif //__SERIAL_CMD__
