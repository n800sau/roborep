#wget http://localhost:9000/cam/stream.mjpg
#wget http://rpi2.local:9000/cam/stream.mjpg
ffmpeg -i stream.mjpg -c:v vp9 -c:a libvorbis stream.mkv
