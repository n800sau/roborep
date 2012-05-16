#ifndef __SERIAL_CMD__

#define __SERIAL_CMD__

void serialSetup();
int processCmdline(String &cmdline, String c_args[], int n);
int getIntVal(const String &strval);
void readSerial(String &c_args[], int max_c_args);
void readI2C(String &c_args[], int max_c_args);

#endif //__SERIAL_CMD__
