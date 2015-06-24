ps -Af|grep -E 'app.fcgi'|grep -v grep|awk '{print $2}'|xargs kill
