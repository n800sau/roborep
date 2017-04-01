#DISPLAY=:5 roslaunch rtabmap_ros demo_robot_mapping.launch rviz:=false rtabmapviz:=true
#DISPLAY=:5 roslaunch rtabmap_stereo_ui.launch &> rtabmap.log
#DISPLAY=:0 roslaunch rtabmap_ros demo_stereo_outdoor.launch
#DISPLAY=:5 roslaunch rtabmap_ros rtabmap.launch rviz:=false rtabmapviz:=true &> rtabmap.log
#DISPLAY=:5 roslaunch rtabmap_ros rtabmap.launch rviz:=false rtabmapviz:=false &> rtabmap.log
#DISPLAY=:5 roslaunch rtabmap_ros rgbd_mapping.launch rviz:=false rtabmapviz:=false &> rtabmap.log
#roslaunch rtabmap_ros rtabmap.launch rviz:=false rtabmapviz:=true
roslaunch rtabmap_ros rtabmap.launch rviz:=false rtabmapviz:=false localization:=false &> rtabmap.log
#DISPLAY=:5 roslaunch rtabmap_ros rgbd_mapping.launch  rviz:=false rtabmapviz:=true
#DISPLAY=:5 roslaunch rtabmap_custom.launch rviz:=false rtabmapviz:=true &> rtabmap.log
#roslaunch rtabmap_custom.launch rviz:=false rtabmapviz:=true &> rtabmap.log
