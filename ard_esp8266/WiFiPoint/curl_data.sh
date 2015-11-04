curl -s --trace - --data 'T=22.00&H=33.00&V=3.56&TIME=1888888888ID=dtest' http://sensgate.local/dht11 &>curl_data.log
echo $?
