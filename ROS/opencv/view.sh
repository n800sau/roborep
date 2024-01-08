#!/bin/sh

#rosrun image_view image_view image:=/image_converted

#rosrun image_view image_view image:=$1

#rosrun image_view image_view image:=/camera/depth/image

#rosrun image_view image_view image:=/depth/image_raw

#rosrun image_view image_view image:=/camera/depth_registered/image_raw
#rosrun image_view image_view image:=/camera/depth_registered/hw_registered/image_rect_raw

#rosrun image_view image_view image:=/camera/rgb/image_raw
#rosrun image_view image_saver image:=/camera/depth/cloud_image
#rosrun image_view image_saver image:=/oculus2wd/processed_points
#rosrun image_view image_saver image:=/oculus2wd/processed_points_image
#rosrun image_view image_saver image:=/camera/depth/cloud_image
#rosrun image_view image_view image:=/camera/depth_registered/hw_registered/image_rect_raw

#rosrun image_view image_view image:=$1

#rosrun image_view disparity_view image:=/camera/depth/disparity
#rosrun image_view disparity_view image:=/camera/depth_registered/disparity
rosrun image_view image_view image:=/oculus2wd/image_motion_nd
#rosrun image_view image_view image:=/camera/rgb/image_raw theora
#rosrun image_view image_view image:=/camera/depth_registered/cloud_image
#rosrun image_view image_view image:=/image_circle
#rosrun image_view image_view image:=/extract_line/image
#rosrun image_view image_view image:=/oculus2wd/signal_image theora

#rosrun image_view image_view image:=/oculus2wd/signal_image_nd

#rosrun image_view image_view image:=/camera/rgb/image_raw
#rosrun image_view image_view image:=/oculus2wd/image_motion1_nd
#rosrun image_view image_view image:=/oculus2wd/signal_image theora

#rosrun image_view image_view image:=/oculus2wd/signal_image_nd

#rosrun image_view image_view image:=/camera/rgb/image_raw
#rosrun image_view image_view image:=/ot/camera/image_raw _image_transport:=compressed
