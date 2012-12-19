#!/bin/sh

../../pi/utils/reset_cam.sh

## ABSOLUTE path to the spawn-fcgi binary
SPAWNFCGI="/usr/bin/spawn-fcgi" 

## ABSOLUTE path to the PHP binary
FCGIPROGRAM="/home/n800s/bin/server" 

## bind to tcp-port on localhost
#FCGIPORT="1026" 

## bind to unix domain socket
FCGISOCKET="/home/n800s/www/bon.sock" 

FCGIPID="/home/n800s/www/bon.pid" 

## IP adresses where PHP should access server connections
## from
FCGI_WEB_SERVER_ADDRS="127.0.0.1,192.168.1.25" 

# allowed environment variables separated by spaces
ALLOWED_ENV="USER" 

## if this script is run as root switch to the following user
USERID=www-data
GROUPID=www-data

#exec $SPAWNFCGI -a $FCGI_WEB_SERVER_ADDRS -p $FCGIPORT -f $FCGIPROGRAM -u $USERID -g $GROUPID
exec sudo $SPAWNFCGI -a $FCGI_WEB_SERVER_ADDRS -s $FCGISOCKET -f $FCGIPROGRAM -u $USERID -g $GROUPID -P $FCGIPID

