esp=192.168.1.97
wget --quiet http://${esp}:8080/command?params=info -O -
echo
echo FREE
wget --quiet http://${esp}:8080/free -O -
echo
echo VCC
wget --quiet http://${esp}:8080/vcc -O -
echo
