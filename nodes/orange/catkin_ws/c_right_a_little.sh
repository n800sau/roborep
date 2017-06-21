#rostopic pub -1 /ot/command fchassis_msgs/command '{mcommand: mright, rpwr: 80, rfwd: false, timeout: 2}'
rosservice call /ot/exec_command '{mcommand: mright, rPwr: 20, rFwd: false, timeout: 1}'
