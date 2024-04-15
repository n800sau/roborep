source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#../../ino/set_stm32_mode/set_pmode.py

#~/stuff/work/roborep/ard_esp8266/io/isp_mode.sh

#DEV=/dev/ttyUSB0
#platformio run -t upload --upload-port `readlink "$DEV"` -e maple &>upload.log

#~/stuff/work/roborep/ard_esp8266/io/work_mode.sh
#platformio run -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV -e demo_f030f4
platformio run -v -t upload
#stm32flash -g 0x08000000 -b 115200 -w ".pio/build/maple/firmware.bin" "/dev/ttyUSB1"
#/usr/bin/stm32flash -g 0x08000000 -i 'rts,-dtr,dtr' -b 115200 -w ".pio/build/maple/firmware.bin" "$DEV"
#/usr/bin/stm32flash -g 0x08000000 -R -i 'rts,dtr,-dtr:rts,-dtr,dtr' -b 115200 -w ".pio/build/maple/firmware.bin" "$DEV"

# flasher working with usb-serial rts -> 10K -> BOOT0 <- 10K <- GND, and DTR -> RESET <- 3.3v
# pip install -U stm32loader

#platformio run &> upload.log && \
#stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/stm32/firmware.bin" &>>upload.log


#platformio run -t upload --upload-port $DEV

#platformio run -e maple && \
#stm32loader -a 0x08000000 -e -w -f F1 -p "$DEV" ".pio/build/maple/firmware.bin"
echo $?

#sleep 1
#../../ino/set_stm32_mode/set_wmode.py
