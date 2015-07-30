ps -Af|grep -E 'fserver'|grep -v grep|awk '{print $2}'|xargs kill
