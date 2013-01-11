#!/bin/sh

start-stop-daemon --start --user n800s --make-pidfile --pidfile /home/n800s/run/camd.pid --background --startas /usr/bin/nohup --chuid n800s -- /home/n800s/work/sourceforge/robotarr-code/lib/CMUcam4/camTr &
