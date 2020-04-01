PID=`ps -eo pid,command | grep 'MQsensors/udp_recv.py'  | grep -v grep | awk '{print $1}'`
kill $PID
echo $?
