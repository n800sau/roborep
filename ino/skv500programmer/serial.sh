source vars.sh
#ino serial &> serial.log
#ino serial
picocom -b 115200 $DEV &>serial.log
#picocom -b 115200 $DEV

