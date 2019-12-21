source vars.sh
#../../bin/miniterm.py -p $DEV -b 115200 &> serial.log
../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0

# connect to socat virtual tty
#../../bin/miniterm.py -p $HOME/dev/ttyV0 -b 115200
