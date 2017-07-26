#ffmpeg -y -i data/input/hubee.avi -deinterlace -vf scale=214:120 -an -r 10 data/input/v.webm &>to_webm.log
ffmpeg -y -i ~/work/eye_work/vids/*.avi -deinterlace -an -r 10 data/fulls/v.webm &>to_webm_full.log
ffmpeg -y -i ~/work/eye_work/vids/*.avi -deinterlace -vf scale=120:90 -an -r 10 data/input/v.webm &>to_webm_input.log

