#rostopic pub -1 /fchassis/command fchassis_msgs/command '{mcommand: mright, rpwr: 80, rfwd: false, timeout: 2}'
rosservice call /exec_command '{mcommand: mright, rPwr: 60, rFwd: false, timeout: 4}'
