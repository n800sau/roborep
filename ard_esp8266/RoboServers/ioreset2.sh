esp=192.168.1.97
wget --quiet http://${esp}:8080/mode/4/o -O -
wget --quiet http://${esp}:8080/digital/4/0 -O -
sleep 1
wget --quiet http://${esp}:8080/digital/4/1 -O -
