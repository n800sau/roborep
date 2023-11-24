DEV=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A601EONT-if00-port0
python2 baudrate.py -a -p $DEV &>baudrate.log
echo $?
