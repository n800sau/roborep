DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
#DEV=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00S7LB-if00-port0

#SKETCH=`basename $PWD`

#echo $SKETCH
#tmp=/tmp/esp8266/$SKETCH
#BIN="${tmp}/${SKETCH}.cpp.bin"

#BIN=.pioenvs/esp01/firmware.bin

#/opt/arduino-1.6.6/hardware/esp8266com/esp8266/tools/esptool -vv -cd ck -cb 115200 -cp $DEV -ca 0x00000 -cf $BIN

#/opt/arduino-1.6.6/hardware/esp8266com/esp8266/tools/esptool -vv -cd ck -cb 115200 -cp $DEV -ca 0x00000 -cf $BIN >upload.log

echo $PREFIX

#~/.platformio/packages/tool-esptoolpy/esptool.py --before default_reset --chip esp8266 --port $DEV --baud 115200 write_flash 0x0 .pio/build/esp01/firmware.bin


platformio run -v -t upload --upload-port $DEV
#platformio run -v -t upload --upload-port $DEV &>uplpoad.log
echo $?

