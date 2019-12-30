source vars.sh

#./bumpRTS $DEV
../../bin/miniterm.py -p $DEV -b 115200 --rts=0 --dtr=0 &> serial.log
#../../bin/miniterm.py -p $DEV -b 115200 --exit-char=0x1b --rts=0 --dtr=1 &> serial.log
#picocom -b 115200 $DEV &>serial.log
#picocom -b 115200 $DEV

#picocom -b 57600 $DEV

#platformio run -t monitor --upload-port $DEV &>serial.log

