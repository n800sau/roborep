#!/bin/sh



case "$1" in
  start)
		rfcomm bind 1 00:12:01:12:00:52 3
  ;;
  stop)
		rfcomm unbind 1 00:12:01:12:00:52
  ;;
  restart|force-reload)
        $0 stop
        sleep 1
        $0 start
  ;;
  status)
        rfcomm
  ;;
  *)
        echo "Usage: $N {start|stop|restart|status}" >&2
        exit 1
        ;;
esac
