#echo '{"cmd": "send_full_data", "interval": 100, "count": 20}' | nc -u -p7980 115.70.59.149 7980
#echo '{"cmd": "send_full_data", "interval": 100, "count": 20}' | test_udp/updcli 192.168.1.24 7880
echo '{"cmd": "send_full_data", "interval": 100, "count": 20}' | test_udp/updcli 115.70.59.149 7980
