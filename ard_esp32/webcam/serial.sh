source vars.sh

#../../bin/miniterm.py -p $DEV -b 115200 --rts=0 --dtr=0 &> serial.log
../../bin/miniterm.py -p $DEV -b 115200 --rts=0 --dtr=0

#platformio run -t monitor --upload-port $DEV &>serial.log

