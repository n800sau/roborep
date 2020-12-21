source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

# flasher working with usb-serial rts -> 10K -> BOOT0 <- 10K <- GND, and DTR -> RESET <- 10k <- 3.3v
# pip install -U stm32loader

#platformio run &> upload.log && \
#stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin" &>upload.log

platformio run && \
stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin"

echo $?
