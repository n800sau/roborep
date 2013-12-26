#!/bin/sh

PIDFILE=/home/n800s/run/roscore.pid
DAEMON=`which roscore`

PPID=`pidof -x -o %PPID ${DAEMON}`
if [ ! -z "$PPID" ]; then
	dpid=`cat $PIDFILE`
	for p in $PPID
	do
		if [ $p -eq $dpid ]
		then
			echo "Running as $p"
			exit 0
		fi
	done
	
fi
echo "Not running"
exit 1
