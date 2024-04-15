source vars.sh

#platformio run -t upload --upload-port $DEV &>upload.log
platformio run -t upload --upload-port $DEV

#/home/n800s/.platformio/packages/tool-rp2040tools/rp2040load -v -D .pio/build/pico/firmware.elf

echo $?


