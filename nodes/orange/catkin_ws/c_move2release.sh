#rostopic pub -1 /ot/command fchassis_msgs/command '{mcommand: move2release, pwr: 50, fwd: false, timeout: 10}'
rosservice call /ot/exec_command '{mcommand: move2release, pwr: 50, fwd: true, timeout: 10}'
