source esp_port.sh
espruino -q -p $PORT --board $BOARD -e 'E.reboot();'
echo $?
