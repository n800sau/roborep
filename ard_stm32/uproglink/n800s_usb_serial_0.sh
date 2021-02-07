DEV=/dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe__STLINK____Firmware_7bc323d__C3DDB9E2-if00
#DEV=/dev/serial/by-id/usb-n800s_UProgLink-if00
#../../bin/miniterm.py -p $DEV -b 115200
../../bin/miniterm.py -p $DEV -b 115200 &> command.log
