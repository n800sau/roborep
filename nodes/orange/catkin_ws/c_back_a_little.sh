#rostopic pub -1 /ot/command fchassis_msgs/command '{mcommand: mboth, lpwr: 50, lfwd: false, rpwr: 50, rfwd: false, timeout: 10}'
rosservice call /ot/exec_command '{mcommand: mboth, lPwr: 20, lFwd: false, rPwr: 20, rFwd: false, timeout: 1}'
