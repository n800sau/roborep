source vars.sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

#platformio run -v -t upload --upload-port $DEV
platformio run -v -t upload
