rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: -2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
#sleep 0
rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
