#mencoder -v rtsp://192.168.1.178:8554/mjpeg/1 -o mplayer_out.avi &>test_mplayer.log
#ffmpeg -i rtsp://192.168.1.178:8554/mjpeg/1 -v debug ffmpeg_out.mp4
#mencoder -v http://192.168.1.178/ -o mplayer_out.avi &>test_mplayer.log
#mplayer -v rtsp://192.168.1.178:8554/mjpeg/1
#mplayer -v http://192.168.1.178/jpg_stream
#ffplay rtsp://192.168.1.178:8554/mjpeg/1 -fps 10
#ffplay http://192.168.1.178/jpg_stream
ffmpeg -i http://192.168.1.178/jpg_stream ffmpeg_out.mp4
echo $?

