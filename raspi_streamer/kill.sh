PID_LIST=`ps aux | grep streamer |grep -v grep| awk '{print $2}'`
echo $PID_LIST
if [ "x$PID_LIST" != "x" ]
then
    kill $PID_LIST
else
    echo Nothing found
fi
