rm source/images/*.png
rm source/labels/*.png
rm source/label_previews/*.png
python3 -u draw_rotate_text.py &>draw_rotate_text.log
echo $?
