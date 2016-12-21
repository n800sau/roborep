#rosbag record -O record_`date -Iminutes`.bag /dn_object_detect/debug_view /dn_object_detect/detected_objects
rosbag record -a -O record_`date -Iminutes`.bag -x '(.*camera.*)|(.*image.*)|(.*rosout.*)|(.*nodelet.*)|(.*multimaster.*)'
