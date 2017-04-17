DISPLAY=:5 python -u video_facial_landmarks.py --shape-predictor shape_predictor_68_face_landmarks.dat -i in.avi &>run_video.log
echo $?
