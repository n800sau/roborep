#!/bin/sh

TCPPORT=9700
SERDEV='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'
#SERDEV='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
stty -F "$SERDEV" 57600
nc -C -l -k $TCPPORT > $SERDEV < $SERDEV
