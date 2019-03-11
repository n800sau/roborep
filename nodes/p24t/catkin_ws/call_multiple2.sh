rosservice call grip '{wvalue: 100}'
sleep 1
rosservice call grip '{wvalue: -1}'
rosservice call yaw '{wvalue: 70}'
sleep 1
rosservice call yaw '{wvalue: -1}'
rosservice call upper '{wvalue: 50}'
sleep 1
rosservice call upper '{wvalue: -1}'
rosservice call lower '{wvalue: 90}'
sleep 1
rosservice call lower '{wvalue: -1}'
