systemctl daemon-reload
systemctl enable dumbproxy.service
systemctl start dumbproxy.service
service rsyslog restart
service logrotate restart
