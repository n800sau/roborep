source vars.sh
#picocom -b 115200 $DEV
../../bin/miniterm.py -p $DEV -b 115200 --exit-char=0x1b
