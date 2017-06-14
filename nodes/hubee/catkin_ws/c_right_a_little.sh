#rostopic pub -1 /ow/command fchassis_msgs/command '{mcommand: mright, rpwr: 80, rfwd: false, timeout: 2}'
rosservice call /ow/exec_command '{mcommand: mright, rPwr: 60, rFwd: false, timeout: 4}'
