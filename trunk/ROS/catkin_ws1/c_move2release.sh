#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: move2release, pwr: 50, fwd: false, timeout: 10}'
rosservice call /exec_command '{mcommand: move2release, pwr: 50, fwd: false, timeout: 10}'
