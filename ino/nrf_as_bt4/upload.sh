source vars.sh

./set_pmode.py
platformio run -t upload --upload-port $DEV &>upload.log
#platformio run -t upload --upload-port $DEV
./set_wmode.py
echo $?
