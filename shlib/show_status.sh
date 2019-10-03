show_status() {
	unset PPID
	if [ -e $PIDFILE ]
	then
		PID=`cat $PIDFILE`
		PPID=`pidof -x -o %PPID ${DAEMON}|xargs -n1 echo|grep $PID`
	fi
	if [ -z "$PPID" ]; then
		echo "Not running"
		exit 1
	else
		echo "Running as\n$PPID"
		stat $PIDFILE|grep 'Access\|Modify\|Change'
		exit 0
	fi
}
