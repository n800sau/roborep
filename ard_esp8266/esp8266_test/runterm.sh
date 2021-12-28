source vars.sh

../../bin/miniterm.py -p $DEV -b 115200
#../../bin/miniterm.py -p $DEV -b 9600 --rts 0 --dtr 0 &>runterm.log
#../../bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
#platformio device monitor --port $DEV -b 115200 &>runterm.log
#../../bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

echo $?
