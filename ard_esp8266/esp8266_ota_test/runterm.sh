source vars.sh

#~/bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 &>runterm.log
~/bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
#~/bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#~/bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

#picocom -b 115200 $DEV

#picocom -b 74880 $DEV

#cat $DEV
echo $?
