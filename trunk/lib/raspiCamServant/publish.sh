#redis-cli publish raspiCamServant '{"cmd": "start_camera"}'
redis-cli publish raspiCamServant '{"cmd": "find_markers", "path": "/home/n800s/public_html/rcam.jpg", "draw_markers": true}'
redis-cli publish raspiCamServant '{"cmd": "stop_camera"}'
