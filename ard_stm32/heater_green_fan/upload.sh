source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#../../ino/set_stm32_mode/set_pmode.py

#DEV=/dev/ttyUSB1
#platformio run -t upload --upload-port $DEV
#platformio run -v -t upload
#stm32flash -g 0x08000000 -b 115200 -w ".pio/build/maple/firmware.bin" "/dev/ttyUSB1"
#/usr/bin/stm32flash -g 0x08000000 -i 'rts,-dtr,dtr' -b 115200 -w ".pio/build/maple/firmware.bin" "$DEV"
#/usr/bin/stm32flash -g 0x08000000 -R -i 'rts,dtr,-dtr:rts,-dtr,dtr' -b 115200 -w ".pio/build/maple/firmware.bin" "$DEV"

# flasher working with usb-serial rts -> 10K -> BOOT0 <- 10K <- GND, and DTR -> RESET <- 3.3v
# pip install -U stm32loader

platformio run &> build.log && \
stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin" &>upload.log
echo $?

#sleep 1
#../../ino/set_stm32_mode/set_wmode.py
