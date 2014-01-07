#!/bin/sh

#rosrun image_view image_view image:=/image_converted

#rosrun image_view image_view image:=$1

#rosrun image_view image_view image:=/camera/depth/image

#rosrun image_view image_view image:=/depth/image_raw

#rosrun image_view image_view image:=/camera/depth_registered/image_raw
#rosrun image_view image_view image:=/camera/depth_registered/hw_registered/image_rect_raw

#rosrun image_view image_view image:=/camera/rgb/image_raw
rosrun image_view image_saver image:=/camera/rgb/image_raw

#rosrun image_view image_view image:=$1
