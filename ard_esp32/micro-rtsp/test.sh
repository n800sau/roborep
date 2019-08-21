#mencoder -v rtsp://192.168.1.178:8554/mjpeg/1 -o mplayer_out.avi &>test_mplayer.log
#ffmpeg -i rtsp://192.168.1.178:8554/mjpeg/1 -v debug ffmpeg_out.mp4 &>test_ffmpeg.log
mencoder -v http://192.168.1.178/ -o mplayer_out.avi &>test_mplayer.log
echo $?

