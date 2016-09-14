#	rosrun image_view image_saver image:=/image_raw _filename_format:="/home/n800s/public_html/last_tmp.png" _save_all_image:=false __name:=image_saver
while true
do
	rosservice call /image_saver/save
	rosservice call /image_saver_raspi/save
	mv /home/n800s/public_html/protopupis/last_tmp.png /home/n800s/public_html/protopupis/last.png
	mv /home/n800s/public_html/protopupis/last_rasp_tmp.png /home/n800s/public_html/protopupis/last_rasp.png
	sleep 2
done
