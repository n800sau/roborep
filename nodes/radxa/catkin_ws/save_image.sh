#	rosrun image_view image_saver image:=/image_raw _filename_format:="/home/n800s/public_html/last_tmp.png" _save_all_image:=false __name:=image_saver
while true
do
	rosservice call /image_saver/save
	mv /home/n800s/public_html/last_tmp.png /home/n800s/public_html/last.png
	sleep 2
done
