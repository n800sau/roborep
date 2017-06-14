#rostopic pub -1 /ot/command fchassis_msgs/command '{mcommand: mstop}'
rosservice call /ot/exec_command '{mcommand: mstop}'
