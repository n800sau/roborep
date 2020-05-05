sudo cp mq_sensors.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable mq_sensors
sudo systemctl start mq_sensors
