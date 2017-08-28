#rostopic pub /ot/command fchassis_msgs/command '{mcommand: walk_around, pwr: 60, timeout: 60}'
rosservice call /ot/exec_command '{mcommand: walk_around, pwr: 60, timeout: 20}'
