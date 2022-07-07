source vars.sh

#platformio run -v -t upload --upload-port $DEV &>upload.log
platformio run -v -t upload --upload-port $DEV
echo $?
