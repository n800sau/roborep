DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
SKETCH=`basename $PWD`

BIN="/tmp/esp8266/${SKETCH}.cpp.bin"
echo $SKETCH

#/opt/arduino-1.6.6/hardware/esp8266com/esp8266/tools/esptool -vv -cd ck -cb 115200 -cp $DEV -ca 0x00000 -cf $BIN
/opt/arduino-1.6.6/hardware/esp8266com/esp8266/tools/esptool -vv -cd ck -cb 115200 -cp $DEV -ca 0x00000 -cf $BIN >upload.log
echo $?

