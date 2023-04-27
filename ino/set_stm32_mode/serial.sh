source vars.sh
#picocom -b 115200 $DEV
#miniterm.py -p $DEV -b 115200 --exit-char=0x1b
platformio device monitor --port $DEV -b 115200 -f time --rts 0 --dtr 0
