#Warning: if timeout then change self.port.timeout = 0.01 to self.port.timeout = 0.1 in SerialClient.py of rosserial_python

#rostopic pub -1 /ot/command fchassis_msgs/command '{mcommand: mboth, lpwr: 50, lfwd: true, rpwr: 50, rfwd: true, timeout: 4}'
rosservice call /ot/exec_command '{mcommand: mboth, lPwr: 20, lFwd: true, rPwr: 20, rFwd: true, timeout: 1}'

