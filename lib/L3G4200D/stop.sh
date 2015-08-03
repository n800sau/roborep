#!/bin/sh

RUN_AS=n800s
PIDFILE=~${RUN_AS}/run/l3g4200d.pid

start-stop-daemon -v --stop --user ${RUN_AS} --pidfile ${PIDFILE} --signal 15 --retry 10
