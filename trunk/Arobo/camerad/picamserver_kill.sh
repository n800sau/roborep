ps -Af|grep -E 'picamserver.py'|grep -v grep|awk '{print $2}'|xargs -n1 kill
