#!/bin/sh

start-stop-daemon --stop --user n800s --pidfile /home/n800s/run/camd.pid --signal 15 --retry 10
