source esp_port.sh
espruino -p $PORT --board $BOARD -o out.js setup_wifi.js &>esp_wifi.log
echo $?
