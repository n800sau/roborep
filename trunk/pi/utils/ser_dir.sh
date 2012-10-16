B0=`cat /sys/devices/virtual/gpio/gpio18/value`
B1=`cat /sys/devices/virtual/gpio/gpio23/value`
#echo $B0 $B1
if [ $B0 == "0" ]
then
    if [ $B1 == "0" ]
    then
        echo arduino
    else
        echo camera
    fi;
else
    if [ $B1 == "0" ]
    then
        echo picaxe
    else
        echo unknown
    fi;
fi
