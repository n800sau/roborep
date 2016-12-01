#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mstop}'
rosservice call /fchassis/exec_command '{mcommand: mstop}'
