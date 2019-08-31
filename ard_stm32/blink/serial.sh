source vars.sh
#picocom -b 115200 $DEV
DEV=/dev/ttyUSB1
../../bin/miniterm.py -p $DEV -b 115200
