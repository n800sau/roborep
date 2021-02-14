source vars.sh

#cd ../set_stm32_mode
#./set_pmode.py
#cd -

#sleep 5
#platformio run -v -t upload -e esp --upload-port $DEV

platformio run -t upload -e esp --upload-port $DEV &>upload.log
#~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp8266 --port $DEV erase_flash
echo $?

#cd ../set_stm32_mode
#./set_wmode.py
#cd -

