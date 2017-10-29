date -Iseconds
rostopic pub -1 /ow/cmd_vel geometry_msgs/Twist '{ linear: {x: 0.9} }'
#sleep 5
#rostopic pub -1 /ow/cmd_vel geometry_msgs/Twist '{ linear: {x: -0.5} }'
#sleep 5
#rostopic pub -1 /ow/cmd_vel geometry_msgs/Twist '{}'
