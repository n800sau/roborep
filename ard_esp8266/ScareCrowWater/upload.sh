source vars.sh

#platformio run -t upload --upload-port $DEV &>upload.log
platformio run -t upload --upload-port $DEV
echo $?

