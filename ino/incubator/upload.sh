source vars.sh

platformio run -v -t upload -e esp01 --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
echo $?


