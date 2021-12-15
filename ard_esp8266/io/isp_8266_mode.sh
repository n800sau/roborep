LOGNAME=isp_8266_mode.log
echo  > $LOGNAME
IO00_PIN=1
RESET_PIN=2
IO15_PIN=3
HOSTPORT=192.168.1.87:8080
wget -q -O - http://$HOSTPORT/mode/$IO00_PIN/o >> $LOGNAME
wget -q -O - http://$HOSTPORT/mode/$RESET_PIN/o >> $LOGNAME
wget -q -O - http://$HOSTPORT/mode/$IO15_PIN/o >> $LOGNAME
wget -q -O - http://$HOSTPORT/digital/$IO00_PIN/0 >> $LOGNAME
wget -q -O - http://$HOSTPORT/digital/$IO15_PIN/0 >> $LOGNAME
wget -q -O - http://$HOSTPORT/digital/$RESET_PIN/0 >> $LOGNAME
wget -q -O - http://$HOSTPORT/digital/$RESET_PIN/1 >> $LOGNAME


