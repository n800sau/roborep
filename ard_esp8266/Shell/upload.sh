source vars.sh

#cd ../set_stm32_mode
#./set_pmode.py
#cd -

#sleep 5
#platformio run -v -t upload -e esp --upload-port $DEV

#python3 "/home/n800s/.platformio/packages/tool-esptoolpy/esptool.py" --chip esp8266 --port "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0" --baud 115200 write_flash 0x0 .pio/build/esp/firmware.bin

platformio run -t upload -e esp --upload-port $DEV &>upload.log
#platformio run -t upload -e esp --upload-port $DEV
#~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp8266 --port $DEV erase_flash
echo $?

#cd ../set_stm32_mode
#./set_wmode.py
#cd -

