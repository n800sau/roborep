source vars.sh

#cd src/data && for file in `ls -A1`; do curl -vvv --digest -F "data=@$file" -F "filename=/$file" --user admin:admin --trace-ascii ../../trace.txt http://192.168.1.89/edit; done && cd -
#cd src/data && for file in `ls -A1`; do http -A digest -a admin:admin pie.dev/digest-auth/httpie/username/password -f POST pie.dev/post data@"$file" http://192.168.1.89/edit; done && cd -
#cd src/data && for file in `ls -A1`; do curl -v --user admin:admin --digest -X PUT -T "$file" "http://192.168.1.89/edit?path=/$file"; done && cd -

ESPTOOL=/home/n800s/.platformio/packages/framework-arduinoespressif8266/tools/esptool/esptool.py
#ESPTOOL=/home/n800s/.platformio/packages/tool-esptoolpy/esptool.py
#ESPTOOL=/home/n800s/.platformio/packages/tool-esptoolpy@1.20600.0/esptool.py

pio run -e esp -v -t uploadfs --upload-port "$DEV" &>uploadfs.log
#python3 $ESPTOOL --chip esp8266 --port "$DEV" --baud 115200 write_flash 3145728 .pio/build/esp/spiffs.bin &>uploadfs.log
#python3 $ESPTOOL --chip esp8266 --port "$DEV" --baud 115200 write_flash 0x300000 .pio/build/esp/spiffs.bin &>uploadfs.log
echo $?
