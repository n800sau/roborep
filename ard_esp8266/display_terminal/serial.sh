source vars.sh

#./bumpRTS $DEV

../../bin/miniterm.py -p $DEV -b 115200 --rts=0 --dtr=0
#../../bin/miniterm.py -p $DEV -b 115200 --rts=0 --dtr=0 &> serial.log
#../../bin/miniterm.py -p $DEV -b 115200 --exit-char=0x1b --rts=0 --dtr=1 &> serial.log

#platformio run -t monitor --upload-port $DEV &>serial.log

