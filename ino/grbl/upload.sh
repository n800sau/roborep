source vars.sh

#wavgat does not work
#platformio run -e LGT8F328P -v -t upload --upload-port $DEV

platformio run -e arduino  -v -t upload --upload-port $DEV
#platformio run -e arduino -t upload --upload-port $DEV &>upload.log
echo $?


