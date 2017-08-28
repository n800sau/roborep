#rostopic pub -1 /ow/command fchassis_msgs/command '{mcommand: mstop}'
rosservice call /ow/exec_command '{mcommand: mstop}'
