LOGNAME=isp_mode.log
echo  > $LOGNAME
# 5 - boot0
# 6 - reset
URI=http://192.168.1.178:8080
wget -q -O - $URI/mode/5/o >> $LOGNAME
wget -q -O - $URI/mode/6/o >> $LOGNAME
wget -q -O - $URI/digital/6/1 >> $LOGNAME
wget -q -O - $URI/digital/5/1 >> $LOGNAME
wget -q -O - $URI/digital/6/0 >> $LOGNAME
#sleep 1
wget -q -O - $URI/digital/6/1 >> $LOGNAME
#sleep 1
#wget -q -O - $URI/digital/5/0 >> $LOGNAME


