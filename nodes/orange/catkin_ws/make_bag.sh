rosbag record -a -O record_`date -Iminutes`.bag -x '(.*camera.*)|(.*image.*)|(.*rosout.*)|(.*nodelet.*)'
