#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mleft, lpwr: 50, lfwd: false, timeout: 1}'
rosservice call /exec_command '{mcommand: mleft, lPwr: 60, lFwd: false, timeout: 4}'
