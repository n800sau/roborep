#rostopic pub -1 /ow/command fchassis_msgs/command '{mcommand: mleft, lpwr: 50, lfwd: false, timeout: 1}'
rosservice call /ow/exec_command '{mcommand: mleft, lPwr: 60, lFwd: true, timeout: 4}'
