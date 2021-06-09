LOGNAME=isp_mode.log
echo  > $LOGNAME
# 5 - boot0
# 6 - reset
wget -q -O - http://192.168.1.87/mode/5/o >> $LOGNAME
wget -q -O - http://192.168.1.87/mode/6/o >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/6/1 >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/5/1 >> $LOGNAME
wget -q -O - http://192.168.1.87/digital/6/0 >> $LOGNAME
#sleep 1
wget -q -O - http://192.168.1.87/digital/6/1 >> $LOGNAME
#sleep 1
#wget -q -O - http://192.168.1.87/digital/5/0 >> $LOGNAME


