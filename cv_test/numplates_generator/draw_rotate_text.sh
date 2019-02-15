rm source/images/*.png
rm source/labels/*.png
python3 draw_rotate_text.py &>draw_rotate_text.log
echo $?
