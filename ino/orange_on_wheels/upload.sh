source vars.sh

#ino upload
#ino upload &> upload.log

platformio run -t upload --upload-port $DEV &>upload.log
echo $?


