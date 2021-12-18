source vars.sh

#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0 &>runterm.log
../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#../../bin/miniterm.py -p $DEV -b 4800 --rts 0 --dtr 0
#platformio device monitor -b 115200 --exit-char=0x1b &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

echo $?
