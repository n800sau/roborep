source vars.sh
#miniterm.py -p $DEV -b 115200 &> serial.log
miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
