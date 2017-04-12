echo  > run.log
#SRCPATH=/home/n800s/sshfs/asus/root/rus_hard/n800s/mydvd/pics_sony/2014-01_01
SRCPATH=/home/n800s/sshfs/asus/root/rus_hard/n800s/mydvd/pics_sony/2015-05-17_bushwalk
#for IMG in images/*
for IMG in "$SRCPATH"/*
do

echo $IMG >>run.log
DISPLAY=:5 python -u facial_landmarks.py --shape-predictor shape_predictor_68_face_landmarks.dat --image "$IMG" &>>run.log
echo $?

done

