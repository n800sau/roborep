esp=192.168.1.97
wget --quiet http://${esp}:8080/command?params=info -O -
wget --quiet http://${esp}:8080/message -O -
echo FREE
wget --quiet http://${esp}:8080/free -O -
echo VCC
wget --quiet http://${esp}:8080/vcc -O -
