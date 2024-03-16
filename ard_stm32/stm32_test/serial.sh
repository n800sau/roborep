source vars.sh
#miniterm.py -p $DEV --dtr 0 -b 115200 &> serial.log
platformio device monitor --port $DEV -b 115200 -f time --rts 0 --dtr 0
# &>serial.log
