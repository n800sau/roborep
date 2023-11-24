source ${HOME}/work/roborep/nodes/p24t/catkin_ws/install/setup.bash
rm -rf lib/ros_lib
rosrun rosserial_arduino make_libraries.py lib
#rosrun rosserial_client make_libraries lib

