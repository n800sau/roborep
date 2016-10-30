#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mboth, lpwr: 50, lfwd: true, rpwr: 50, rfwd: true, timeout: 4}'
rosservice call /exec_command '{mcommand: mboth, lPwr: 60, lFwd: true, rPwr: 60, rFwd: true, timeout: 4}'
