source vars.sh

sudo sh reset.sh

#picocom -b 115200 $DEV &>serial.log
#picocom -b 9600 $DEV
#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>serial.log
../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0

#picocom -b 57600 $DEV

