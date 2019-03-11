PID=`ps -eo pid,command | grep 2ftp  | grep -v grep | awk '{print $1}'`
if [ "x$PID" != "x" ]
then
	kill $PID
else
	echo Not found
fi
echo $?
