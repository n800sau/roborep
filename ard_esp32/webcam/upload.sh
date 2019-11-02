source vars.sh

platformio run -t upload
#platformio run -t upload --upload-port $DEV
#platformio run -v -t upload --upload-port $DEV &>upload.log
#platformio run -t upload --upload-port myesp32.local &>upload.log
#~/.platformio/packages/framework-arduinoespressif32/tools/espota.py -r -i esp32cam.local -f .pio/build/main/firmware.bin -d -p 3232 &>upload.log
echo $?


