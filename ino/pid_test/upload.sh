source vars.sh

#platformio run -e LGT8F328P -v -t upload --upload-port $DEV
platformio run -e LGT8F328P -v -t upload --upload-port $DEV &>upload.log

#platformio run -e arduino_uno  -v -t upload --upload-port $DEV
#platformio run -e arduino_uno -t upload --upload-port $DEV &>upload.log
echo $?


