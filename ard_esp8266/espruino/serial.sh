source vars.sh
#../../bin/miniterm.py -p $DEV -b 115200 &> serial.log
miniterm.py $DEV 115200 --rts=0 --dtr=0
#miniterm.py $DEV 115200

#../../bin/miniterm.py -p $DEV -b 460800 --exit-char=0x1b

