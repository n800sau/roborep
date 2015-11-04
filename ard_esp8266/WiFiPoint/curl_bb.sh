curl -s -v --trace - --data 'T=22.00&H=33.00&V=3.56&TIME=1444781951&ID=test' http://192.168.2.80:5580/sensors/accept.php &>curl_bb.log
echo $?
