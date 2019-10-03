#rostopic pub /ow/command ow_msgs/command '{mcommand: walk_around, pwr: 60, timeout: 60}'
rosservice call /ow/exec_command '{mcommand: walk_around, pwr: 60, timeout: 40}'
