source esp_port.sh
espruino -p $PORT -b 115200 --board $BOARD -e 'print(require("Storage").list());' &>esp_ls.log
echo $?
