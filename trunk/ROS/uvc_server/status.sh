#!/bin/sh

PIDFILE=/home/n800s/run/uvc_server.pid
DAEMON="uvc_camera uvc_camera_node _device:=/dev/v4l/by-id/usb-BISON_Corporation_Lenovo_EasyCamera-video-index0"

PPID=`pidof -x -o %PPID ${DAEMON}`
if [ -z "$PPID" ]; then
                     echo "Not running"
                     exit 1
             else
                     echo "Running as $PPID"
                     exit 0
             fi
