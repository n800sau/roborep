source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#../../ino/set_stm32_mode/set_pmode.py

DEV=/dev/ttyUSB1
#platformio run -t upload --upload-port $DEV
#platformio run -v -t upload
#stm32flash -g 0x08000000 -b 115200 -w ".pio/build/maple/firmware.bin" "/dev/ttyUSB1"
stm32flash -g 0x08000000 -b 115200 -w ".pio/build/maple/firmware.bin" "$DEV"

#sleep 1
#../../ino/set_stm32_mode/set_wmode.py
