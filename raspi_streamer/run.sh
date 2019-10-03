#!/bin/sh

PORT=9000
uwsgi --plugin python --http-socket 0.0.0.0:$PORT --py-autoreload 3 --ini uwsgi.ini \
	--pidfile=${HOME}/tmp/uwsgi_$PORT.pid --pyargv ""
