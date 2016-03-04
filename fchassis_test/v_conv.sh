# install gpac
# apt-get install gpac

MP4Box -add v.mp4#video -raw 1 -new vtest
MP4Box -add vtest_track1.h264:fps=5 -new new_v.mp4
