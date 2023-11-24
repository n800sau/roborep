rostopic pub -1 /command std_msgs/String 'set m2vel 0.03'
rostopic pub -1 /command std_msgs/String 'go'
./stop.sh
