source vars.sh

#platformio run -v
platformio run -v -t upload --upload-port $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
echo $?


