source vars.sh

#platformio run -v -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log

BIN=.pio/build/esp01/firmware.bin

python3 "/home/n800s/.platformio/packages/tool-esptoolpy/esptool.py" --chip esp8266 --port "$DEV" --baud 115200 write_flash 0x0 "$BIN"

echo $?


