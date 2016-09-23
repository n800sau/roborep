ffmpeg -y -i data/input/hubee.avi -deinterlace -vf scale=214:120 -an -r 10 data/input/v.webm
