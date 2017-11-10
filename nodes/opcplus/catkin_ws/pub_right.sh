rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -1.5}}'
#sleep 3
rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
