source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#platformio run && tm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin"

platformio run &> upload.log && stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin" &>upload.log

echo $?