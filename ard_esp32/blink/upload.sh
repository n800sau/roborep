source vars.sh

#platformio run -v -t upload --upload-port $DEV
#platformio run -v -t upload --upload-port $DEV &>upload.log
platformio run -v -t upload --upload-port 192.168.1.151 &>upload.log
echo $?


