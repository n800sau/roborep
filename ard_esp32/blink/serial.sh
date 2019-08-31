source vars.sh

./bumpRTS $DEV
#./reset.py
#picocom -b 115200 $DEV &>serial.log
#picocom -b 115200 $DEV
#platformio device monitor -b 115200 &>serial.log
../../bin/miniterm.py -p $DEV -b 115200 --exit-char=0x1b

#picocom -b 57600 $DEV

