source vars.sh

platformio run -v -e LGT8F328P -t upload --upload-port $DEV
#platformio run -v -e uno -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
echo $?


