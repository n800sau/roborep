source vars.sh

platformio run -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
echo $?


