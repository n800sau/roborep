esp=192.168.1.97
wget --quiet http://${esp}:8080/mode/4/o -O -
echo
echo 0
wget --quiet http://${esp}:8080/digital/4/0 -O -
echo
sleep 1
echo 1
wget --quiet http://${esp}:8080/digital/4/1 -O -
echo
