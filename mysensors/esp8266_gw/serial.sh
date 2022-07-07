source vars.sh

#miniterm.py -p $DEV -b 115200 2> error.log|grep -v '0,sg=0,ft=0,st=OK' >serial.log
miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>serial.log
#miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0
#miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 &>serial.log
#platformio device monitor -b 115200 &>serial.log
#platformio device monitor -b 115200

#picocom -b 115200 $DEV

#picocom -b 74880 $DEV

#cat $DEV
echo $?
