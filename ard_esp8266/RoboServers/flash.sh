PWD=`pwd`
PREFIX="/tmp/esp8266/`basename $PWD`.cpp.elf"
echo $PREFIX
stty -echo
esptool.py elf2image $PREFIX && \
esptool.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 write_flash 0x00000 ${PREFIX}-0x00000.bin 0x10000 ${PREFIX}-0x10000.bin
echo $?
stty echo
