source vars.sh

#export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/n800s/.platformio/packages/tool-stlink/lib

../../ino/set_stm32_mode/set_pmode.py &>upload.log

DEV=`readlink $DEV`
DEV=`basename $DEV`
platformio run -t upload --upload-port $DEV &>>upload.log
#platformio run -t upload --upload-port $DEV
echo $?
#sleep 1
../../ino/set_stm32_mode/set_wmode.py &>>upload.log

#./serial.sh
