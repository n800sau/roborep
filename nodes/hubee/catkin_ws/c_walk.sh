#rostopic pub /fchassis/command fchassis_msgs/command '{mcommand: walk_around, pwr: 60, timeout: 60}'
rosservice call /fchassis/exec_command '{mcommand: walk_around, pwr: 60, timeout: 20}'
