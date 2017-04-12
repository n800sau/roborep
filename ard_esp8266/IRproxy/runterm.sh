DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
#DEV=/dev/ttyUSB0

#~/bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0 &>runterm.log
~/bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 &>runterm.log
#~/bin/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
#~/bin/miniterm.py -p $DEV -b 74880 --rts 0 --dtr 0

#picocom -b 115200 $DEV
#cat $DEV
echo $?
