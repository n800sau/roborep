export DISPLAY=:5
#roslaunch main.launch|tee run.log
roslaunch main.launch &>run.log
#roslaunch main.launch
echo $?

