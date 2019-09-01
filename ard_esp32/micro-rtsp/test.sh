#mencoder -v rtsp://192.168.1.178:8554/mjpeg/1 -ovc copy -nosound -o mplayer_out.avi &>test_mplayer.log
#ffmpeg -i rtsp://192.168.1.178:8554/mjpeg/1 -y -v debug ffmpeg_out.mp4 &>test_ffmpeg.log
ffmpeg -i http://192.168.1.178/jpg_stream -y -v debug ffmpeg_out.mp4 &>test_ffmpeg.log
#ffmpeg -i http://192.168.1.178/jpg_stream -y -v debug ffmpeg_out_1.mp4 &>test_ffmpeg_1.log
#mencoder -v http://192.168.1.178/ -o mplayer_out.avi &>test_mplayer.log
#wget http://192.168.1.178/jpg -O out.jpg &>wget.log

echo $?

