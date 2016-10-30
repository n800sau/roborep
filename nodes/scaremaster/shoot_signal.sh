PID=`ps -eo pid,command | grep raspistill  | grep -v grep | awk '{print $1}'`
kill -s SIGUSR1 $PID
echo $?
