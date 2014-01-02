#!/bin/sh

PIDFILE=/home/n800s/run/uvc_server.pid
DAEMON="uvc_camera uvc_camera_node _device:=/dev/v4l/by-id/usb-BISON_Corporation_Lenovo_EasyCamera-video-index0"

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
