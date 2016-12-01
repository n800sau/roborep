#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mboth, lpwr: 50, lfwd: false, rpwr: 50, rfwd: false, timeout: 10}'
rosservice call /fchassis/exec_command '{mcommand: mboth, lPwr: 80, lFwd: false, rPwr: 80, rFwd: false, timeout: 10}'
