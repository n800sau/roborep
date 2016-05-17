DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

platformio run -t upload --upload-port $DEV &>upload.log
#platformio run -t upload --upload-port $DEV
echo $?

