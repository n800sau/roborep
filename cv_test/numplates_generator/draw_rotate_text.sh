rm source/images/*.png
rm source/labels/*.png
rm source/label_previews/*.png
rm source/xml/*.xml
python3 -u draw_rotate_text.py &>draw_rotate_text.log
echo $?
