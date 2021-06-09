source vars.sh

#platformio run -v -t upload --upload-port $DEV
platformio run -v -t upload --upload-port $DEV &>upload.log
echo $?


