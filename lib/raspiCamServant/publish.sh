HOST=hubee

redis-cli -h $HOST del image
redis-cli -h $HOST del raspiCamServant.js.obj
redis-cli -h $HOST publish raspiCamServant '{"cmd": "start_camera", "width": 320, "height": 240}'
#redis-cli -h $HOST publish raspiCamServant '{"cmd": "find_markers", "path": "/home/n800s/public_html/rcam.jpg", "draw_markers": true}'
#redis-cli -h $HOST publish raspiCamServant '{"cmd": "make_shot", "path": "/home/n800s/public_html/rcam.jpg"}'
#redis-cli -h $HOST publish raspiCamServant '{"cmd": "make_shot", "path": "/home/n800s/work/robotarr-code/lib/raspiCamServant/rcam.jpg"}'

for i in 1
do

	redis-cli -h $HOST publish raspiCamServant '{"cmd": "make_shot", "path": "redis:image"}'

	# wait for reply
	time sh -c "
		redis-cli -h $HOST blpop raspiCamServant.js.obj 10
		# get image
		redis-cli -h $HOST get image >~/public_html/hubee_$i.jpg
	"

done

redis-cli -h $HOST publish raspiCamServant '{"cmd": "stop_camera"}'
