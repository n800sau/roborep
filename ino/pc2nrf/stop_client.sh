#!/bin/sh

PIDFILE=${HOME}/run/nrfredis_client.pid

start-stop-daemon -v --stop --user n800s --pidfile ${PIDFILE} --signal 15 --retry 10
