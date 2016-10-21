#rostopic pub /fchassis/command fchassis_msgs/command '{mcommand: walk_around, pwr: 60, timeout: 60}'
rosservice call /exec_command '{mcommand: walk_around, pwr: 50, timeout: 60}'
