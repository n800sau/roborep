source esp_port.sh
SCRIPT=script.js
espruino -p $PORT --board $BOARD -o out.js "$SCRIPT" --config SAVE_ON_SEND=1 &>esp_intf.log
echo $?
