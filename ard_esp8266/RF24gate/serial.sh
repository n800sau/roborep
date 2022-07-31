source vars.sh

#miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>serial.log
miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0
#platformio device monitor -b 115200 &>runterm.log
#platformio device monitor -b 115200

#picocom -b 115200 $DEV

#picocom -b 74880 $DEV

#cat $DEV
echo $?
