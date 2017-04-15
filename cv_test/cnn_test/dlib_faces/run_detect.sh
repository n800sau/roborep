echo  > run_detect.log
#for IMG in images/*
for IMG in /home/n800s/sshfs/asus/root/rus_hard/n800s/mydvd/pics_sony/2015-05-24_medieval/*
do

echo $IMG >>run_detect.log
DISPLAY=:5 python -u detect_face_parts.py --shape-predictor shape_predictor_68_face_landmarks.dat --image "$IMG" &>>run_detect.log
echo $?

done

