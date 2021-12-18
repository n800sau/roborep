i=0
VEXT=avi
#VEXT=MP4
#SCALE=240
SCALE=360
#flist=`find ~/work/car_video/test/*.${VEXT} | sort -R`
#flist=`find ~/work/licence_plate/output/*.${VEXT} | sort -R`
#flist=`find ~/work/test_cv3/text-detection/output_vids/*.${VEXT} | sort -R`
#flist=`find ~/work/keras-maskrcnn/output/video | sort -R`
#flist=`find ~/work/predict/keras_Realtime_Multi-Person_Pose_Estimation/videos/outputs/*.mp4`
#flist="`find  ~/work/roborep/openscad/*.mp4 | sort -R`"
#flist="`find ~/work/roborep/cv_test/timelapse/ -type f -name '*.avi' | sort -R`"
#flist="`find  ~/work/spectra/*.avi | sort -R`"
#flist="`find  ~/'panorama_jpg/Panorama-Stiching/videos/'*.mp4 | sort -R`"
#flist="`find  ~/work/selivan/bitbucket/uni_dnn/data/output_vids.segnet/*.mp4 | sort -R`"
#flist="`find ~/work/selivan/bitbucket/drobilka_t/image-segmentation-keras/output -type f -name '*.mp4' -o -name '*.avi' | sort -R`"
flist="`find ~/work/selivan/bitbucket/drobilka_t/image-segmentation-keras/dpt_output -type f -name '*.mp4' -o -name '*.avi' | sort -R`"
#flist="`find ~/tmp/vids/*.mp4 | sort -R`"


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
