DISPLAY=:0 rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 --approximate=0.1 \
  right:=/stereo_camera/right/image_raw_throttle left:=/stereo_camera/left/image_raw_throttle \
  right_camera:=/stereo_camera/right left_camera:=/stereo_camera/left

