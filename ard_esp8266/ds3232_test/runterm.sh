#!/bin/bash

DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
#DEV=/dev/ttyUSB0

/opt/Espressif/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0 >runterm.log
#/opt/Espressif/miniterm.py -p $DEV -b 115200 --rts 0 --dtr 0
