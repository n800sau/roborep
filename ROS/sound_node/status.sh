#!/bin/sh

PIDFILE=/home/n800s/run/sound_node.pid
DAEMON="python soundplay_node.py"

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
