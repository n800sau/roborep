for i in 1 2 3
do
rosservice call /ow/twist_scan "{scan_id: 1, scan_attempts: $i}" &> twist_scan$i.log
echo $?
#calc "`grep 34 twist_scan.log|wc -l` / `grep angle twist_scan.log` * 100" ; echo '%' >> twist_scan.log
done

