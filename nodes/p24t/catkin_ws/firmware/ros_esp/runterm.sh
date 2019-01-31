source vars.sh

../../../../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 --exit-char=0x1b &>runterm.log
#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 --exit-char=0x1b
#platformio device monitor -b 115200 --exit-char=0x1b &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 --exit-char=0x1b

echo $?
