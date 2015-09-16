#!/bin/sh

cd `dirname $0`

RUN_AS=n800s
PIDFILE=~/run/raspiCamServant.pid

start-stop-daemon -v --stop --user ${RUN_AS} --pidfile ${PIDFILE} --signal 15 --retry 10
