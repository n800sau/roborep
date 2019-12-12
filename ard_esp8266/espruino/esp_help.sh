source esp_port.sh
espruino -p $PORT -b 115200 --listconfigs --board $BOARD &>esp_help.log
echo $?
