rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: -1.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
sleep 5
rostopic pub -1 /opcplus/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

#rostopic pub -1 /opcplus/lwheel_desired_rate std_msgs/Int16 -- -1000
#rostopic pub -1 /opcplus/rwheel_desired_rate std_msgs/Int16 -- 1000

#sleep 5

#rostopic pub -1 /opcplus/lwheel_desired_rate std_msgs/Int16  0 &
#rostopic pub -1 /opcplus/rwheel_desired_rate std_msgs/Int16  0 &
