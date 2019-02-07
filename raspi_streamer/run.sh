#!/bin/sh

PORT=9000
uwsgi --plugin python --http-socket 0.0.0.0:$PORT --py-autoreload 3 \
	--wsgi-file start.py --master --add-header='Connection: Keep-Alive' \
	--threads 2 --processes 2 --pidfile=${HOME}/tmp/uwsgi_$PORT.pid --buffer-size=65535 \
	--uid arch --gid arch --pyargv "" --static-index index.html --check-static static
