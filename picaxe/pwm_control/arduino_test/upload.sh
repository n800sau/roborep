source vars.sh

platformio run -v -e esp -t upload --upload-port $DEV
#platformio run -e esp -t upload --upload-port $DEV &>upload.log
echo $?


