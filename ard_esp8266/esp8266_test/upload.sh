source vars.sh

#~/.local/bin/esptool.py --before no_reset --after soft_reset --chip esp8266 --port "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0" --baud 115200 write_flash 0x0 .pio/build/esp01/firmware.bin
#~/.local/bin/esptool.py --before usb_reset --after soft_reset --chip esp8266 --port "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0" --baud 74880 write_flash 0x0 .pio/build/esp01/firmware.bin

~/work/roborep/ino/set_esp_mode/set_pmode.py

platformio run -v -t upload --upload-port $DEV
#platformio run -v -t upload --upload-port $DEV &>upload.log
echo $?

~/work/roborep/ino/set_esp_mode/set_wmode.py

