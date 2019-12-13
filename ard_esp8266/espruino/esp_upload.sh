source esp_port.sh
espruino -p $PORT --board $BOARD --ohex out.hex --storage .boot0:"$1"
echo $?
