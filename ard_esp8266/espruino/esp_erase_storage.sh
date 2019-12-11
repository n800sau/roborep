source esp_port.sh
espruino -q -p $PORT --board $BOARD -e 'require("Storage").eraseAll();'
echo $?
