source vars.sh

~/work/roborep/ino/set_esp_mode/set_pmode.py

#platformio run -v -t upload --upload-port $DEV
platformio run -v -t upload --upload-port $DEV &>upload.log
echo $?

~/work/roborep/ino/set_esp_mode/set_wmode.py

