echo  > run.log
#for IMG in images/*
for IMG in /home/n800s/sshfs/asus/root/rus_hard/n800s/mydvd/pics_sony/2014-01_01/*
do

echo $IMG >>run.log
DISPLAY=:5 python -u facial_landmarks.py --shape-predictor shape_predictor_68_face_landmarks.dat --image "$IMG" &>>run.log
echo $?

done

