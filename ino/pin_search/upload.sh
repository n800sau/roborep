source vars.sh

platformio run -v -t upload --upload-port $DEV
echo $?
