rostopic pub -1 /rpi3/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 3.5}}'
#sleep 1
rostopic echo -n1 /rpi3/rf2o_laser_odometry/odom_rf2o >topic.log
rostopic pub -1 /rpi3/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
