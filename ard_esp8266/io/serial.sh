source vars.sh

#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>serial.log
#miniterm.py -p $DEV -b 115200
#../../bin/miniterm.py -p $DEV -b 4800 --rts 0 --dtr 0 --raw
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0
#platformio device monitor -p $DEV -b 115200
platformio device monitor -p $DEV -b 115200 --rts 0 --dtr 0

#picocom -b 115200 $DEV

#picocom -b 74880 $DEV

#cat $DEV
echo $?
