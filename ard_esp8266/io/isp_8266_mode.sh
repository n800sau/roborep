LOGNAME=isp_8266_mode.log
echo  > $LOGNAME
IO00_PIN=1
RESET_PIN=2
IO15_PIN=3
wget -q -O - http://192.168.1.87/mode/$IO00_PIN/o >> $LOGNAME
wget -q -O - http://192.168.1.87/mode/$RESET_PIN/o >> $LOGNAME
wget -q -O - http://192.168.1.87/mode/$IO15_PIN/o >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/$IO00_PIN/0 >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/$IO15_PIN/0 >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/$RESET_PIN/0 >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/$RESET_PIN/1 >> $LOGNAME


