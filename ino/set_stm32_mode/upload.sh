source vars.sh

#platformio upload &>upload.log
platformio run -v -t upload --upload-port $DEV &>upload.log
echo $?
