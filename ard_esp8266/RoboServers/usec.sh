esp=192.168.1.97
wget http://${esp}:8080/selser?params=1 --quiet -O -
wget --quiet http://${esp}:8080/message -O -
wget http://${esp}:8080/setbaud?params=19200 --quiet -O -
echo BAUD
wget --quiet http://${esp}:8080/message -O -
