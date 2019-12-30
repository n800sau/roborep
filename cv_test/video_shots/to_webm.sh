i=0
VEXT=avi
#VEXT=MP4
#SCALE=240
SCALE=360
#flist=`find ~/work/car_video/test/*.${VEXT} | sort -R`
#flist=`find ~/work/licence_plate/output/*.${VEXT} | sort -R`
#flist=`find ~/work/test_cv3/text-detection/output_vids/*.${VEXT} | sort -R`
#flist=`find ~/work/keras-maskrcnn/output/video | sort -R`
flist=`find ~/work/predict/keras_Realtime_Multi-Person_Pose_Estimation/videos/outputs/*.mp4`

rm file.lst

for rfname in $flist
do

bname="`basename $rfname .${VEXT}`.webm"
echo $bname >>file.lst
ffmpeg -y -i $rfname -deinterlace -an -r 10 data/fulls/$bname &>to_webm_full.log
echo $?
ffmpeg -y -i $rfname -deinterlace -vf "scale=${SCALE}"':trunc(ow/a/2)*2' -an -r 10 data/input/$bname &>to_webm_input.log
echo $?

i="$(( i + 1 ))"

done
