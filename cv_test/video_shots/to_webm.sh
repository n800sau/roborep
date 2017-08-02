i=0
flist=`find ~/work/eye_work/vids/*.avi | sort -R`

rm file.lst

for rfname in $flist
do

bname="`basename $rfname .avi`.webm"
echo $bname >>file.lst
ffmpeg -y -i $rfname -deinterlace -an -r 10 data/fulls/$bname &>to_webm_full.log
echo $?
ffmpeg -y -i $rfname -deinterlace -vf 'scale=240:trunc(ow/a/2)*2' -an -r 10 data/input/$bname &>to_webm_input.log
echo $?

i="$(( i + 1 ))"

done
