sudo cp iot_server.service /etc/systemd/system
sudo systemctl --system daemon-reload
sudo systemctl enable iot_server
sudo systemctl start iot_server
