DEV=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0

#ino upload
#ino upload &> upload.log

platformio run -t upload --upload-port $DEV
echo $?


