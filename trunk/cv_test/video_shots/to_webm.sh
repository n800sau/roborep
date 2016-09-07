#ffmpeg -y -i data/input/v.MOV -deinterlace -vf scale=214:120 -an -r 10 data/input/v.webm &>to_webm.log
ffmpeg -y -i data/input/v.MOV -deinterlace -vf scale=214:-1 -an -r 10 data/input/v.webm &>to_webm.log
echo $?
