source vars.sh

#platformio run -t upload --upload-port $DEV -e internal1M &>upload.log
#platformio run -t upload --upload-port $DEV -e internal1M
platformio run -t upload --upload-port $DEV -e uno
echo $?
