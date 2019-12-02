#roslaunch rtabmap_ros stereo_mapping.launch rtabmap_args:="--delete_db_on_start" rgbd:=false stereo:=true
#roslaunch rtabmap_ros stereo_mapping.launch stereo_namespace:="/stereo_camera" rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false
#roslaunch rtabmap_ros stereo_mapping.launch rviz:=true rtabmapviz:=false
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" rviz:=true rtabmapviz:=false
