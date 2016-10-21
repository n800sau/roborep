#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mstop}'
rosservice call /exec_command '{mcommand: mstop}'
