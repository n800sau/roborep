source esp_port.sh
espruino -t -p $PORT --board $BOARD -o out.js -e "save();" script.js &>esp_intf.log
echo $?
