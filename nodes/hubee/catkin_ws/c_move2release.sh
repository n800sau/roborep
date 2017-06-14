#rostopic pub -1 /ow/command fchassis_msgs/command '{mcommand: move2release, pwr: 50, fwd: false, timeout: 10}'
rosservice call /ow/exec_command '{mcommand: move2release, pwr: 50, fwd: false, timeout: 10}'
