esp=192.168.1.153
wget http://${esp}:8080/selser?params=1 --quiet -O -
echo
#wget http://${esp}:8080/setbaud?params=19200 --quiet -O -
wget http://${esp}:8080/setbaud?params=115200 --quiet -O -
echo
