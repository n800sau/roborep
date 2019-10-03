source vars.sh

<<<<<<< HEAD
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 &>runterm.log
#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0  --exit-char=0x1b
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 --exit-char=0x1b
=======
#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
#../../bin/miniterm.py -p $DEV -b 250000 --rts 0 --dtr 0
../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 --exit-char=0x1b &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0
#platformio device monitor -b 115200 &>runterm.log
#platformio device monitor -b 115200

#picocom -b 115200 $DEV

#picocom -b 74880 $DEV
>>>>>>> b01701c0c8e8bb7fad9865f0e944a959a63b3a66

#cat $DEV
echo $?
