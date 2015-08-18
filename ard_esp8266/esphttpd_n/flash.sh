#!/bin/bash

PWD=`pwd`
PREFIX="/tmp/esp8266/`basename $PWD`.cpp_"
echo $PREFIX
stty '-echo'
esptool.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 write_flash 0x00000 ${PREFIX}00000.bin 0x40000 ${PREFIX}40000.bin
stty echo
