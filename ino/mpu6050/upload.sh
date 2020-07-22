source vars.sh

platformio run -e LGT8F328P -v -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
echo $?


