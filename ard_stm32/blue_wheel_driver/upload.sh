source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

sudo ./progmode.sh
platformio run -v -t upload --upload-port $DEV &>upload.log
#platformio run -v -t upload --upload-port $DEV
echo $?
#platformio run -v -t upload
sudo ./workmode.sh
