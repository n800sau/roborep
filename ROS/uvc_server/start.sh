#!/bin/sh

PIDFILE=/home/n800s/run/uvc_server.pid
DAEMON="`which rosrun` uvc_camera uvc_camera_node _device:=/dev/v4l/by-id/usb-BISON_Corporation_Lenovo_EasyCamera-video-index0"
start-stop-daemon -v --start --user n800s --make-pidfile --pidfile ${PIDFILE} --background --startas ${DAEMON} --chuid n800s -- ${DAEMON_ARGS}
