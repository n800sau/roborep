source vars.sh

#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0 &>runterm.log
#miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
miniterm.py -p $DEV -b 19200 --rts 0 --dtr 0
#platformio device monitor -b 115200 --exit-char=0x1b &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

echo $?
