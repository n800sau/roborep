PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
#PORT="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00S7LB-if00-port0"

/opt/Espressif/miniterm.py -p "$PORT" -b 115200 --rts 0 --dtr 0 >runterm.log
#/opt/Espressif/miniterm.py -p "$PORT" -b 115200 --rts 0 --dtr 0

#/opt/Espressif/miniterm.py -p "$PORT" -b 115200 >runterm.log
