sudo cp homeassistant@homeassistant.service /etc/systemd/system
sudo systemctl --system daemon-reload
sudo systemctl enable homeassistant@homeassistant
sudo systemctl start homeassistant@homeassistant