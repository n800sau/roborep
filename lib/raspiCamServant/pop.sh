HOST=hubee
echo $?

redis-cli -h $HOST lpop raspiCamServant.js.obj >popped.txt


