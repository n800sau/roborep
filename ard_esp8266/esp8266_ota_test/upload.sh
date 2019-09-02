source vars.sh

<<<<<<< HEAD
~/work/roborep/ino/set_esp_mode/set_pmode.py
#platformio run -v -t upload --upload-port $DEV
platformio run -t upload --upload-port $DEV &>upload.log
~/work/roborep/ino/set_esp_mode/set_wmode.py
=======
platformio run -v -t upload --upload-port $DEV
#platformio run -v -t upload --upload-port $DEV &>upload.log
>>>>>>> b01701c0c8e8bb7fad9865f0e944a959a63b3a66
echo $?


