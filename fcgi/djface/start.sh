#!/bin/sh

#python ./manage.py runfcgi method=prefork host=127.0.0.1 port=1026 pidfile=/tmp/djface.pid
python ./manage.py runfcgi method=prefork socket=/home/n800s/www/robo.sock umask=000 pidfile=/home/n800s/www/robo.pid errlog=`pwd`/errlog outlog=`pwd`/outlog
