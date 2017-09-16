source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#usb-reset $DEV
#platformio run -t upload --upload-port $DEV &>upload.log
#./set_rts1 && \
platformio run -v -t upload --upload-port $DEV
#platformio run -t upload &>upload.log
#platformio run -t upload
echo $?


