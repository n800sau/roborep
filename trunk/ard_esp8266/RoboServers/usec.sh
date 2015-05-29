esp=192.168.1.97
wget http://${esp}:8080/selser?params=1 --quiet -O -
echo
wget http://${esp}:8080/setbaud?params=19200 --quiet -O -
echo
