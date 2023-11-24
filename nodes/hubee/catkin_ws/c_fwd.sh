#rostopic pub -1 /ow/command fchassis_msgs/command '{mcommand: mboth, lpwr: 50, lfwd: true, rpwr: 50, rfwd: true, timeout: 4}'
rosservice call /ow/exec_command '{mcommand: mboth, lPwr: 80, lFwd: true, rPwr: 80, rFwd: true, timeout: 2}'
