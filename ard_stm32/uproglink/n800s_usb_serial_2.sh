DEV=/dev/serial/by-id/usb-n800s_UProgLink-if02
../../bin/miniterm.py -p $DEV -b 115200
#../../bin/miniterm.py -p $DEV --dtr 0 -b 115200 &> serial.log
