PID=`ps -eo pid,command | grep udprecv.py  | grep -v grep | awk '{print $1}'`
kill $PID
echo $?
