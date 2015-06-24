ps -Af|grep -E 'irserver'|grep -v grep|awk '{print $2}'|xargs kill
