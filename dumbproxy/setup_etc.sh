systemctl daemon-reload
systemctl enable dumbproxy.socket
systemctl start dumbproxy.socket
systemctl enable dumbproxy.service
systemctl start dumbproxy.service
service rsyslog restart
