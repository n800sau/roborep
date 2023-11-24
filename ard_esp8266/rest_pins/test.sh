echo > test.log
wget --user admin --password admin -q -O - http://192.168.1.178/mode/o/5 &>>test.log
#wget --user admin --password admin -q -O - http://192.168.1.178/wd/5/1 &>>test.log
wget --user admin --password admin -q -O - http://192.168.1.178/wa/5/128 &>>test.log
echo $?
