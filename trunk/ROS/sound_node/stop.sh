#!/bin/sh

PIDFILE=/home/n800s/run/sound_node.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
