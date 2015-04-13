PWD=`pwd`
SKETCH=`basename $PWD`
echo $SKETCH
rm -rf /tmp/esp8266
/opt/arduino-1.6.1/arduino -v --verify --pref build.path=/tmp/esp8266 --board esp8266com:esp8266:esp01 ${SKETCH}.ino
