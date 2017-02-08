#export DISPLAY=:5
#export OPENNI2_DRIVERS_PATH=/usr/lib/OpenNI2/Drivers
#export ROS_HOME=`pwd`
#roslaunch main_openni2.launch &>run_openni2.log
#roslaunch main_openni2.launch
roslaunch main_openni2.launch depth_registration:=true ir_processing:=true disparity_processing:=true &>run_openni2.log
#roslaunch openni2_launch openni2.launch depth_registration:=true ir_processing:=true &>run_openni2.log

