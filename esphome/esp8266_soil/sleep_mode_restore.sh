mosquitto_pub -h 192.168.1.50 -u user1 -P password1 -r -t moisturesensor/ota_mode -m 'OFF'
sleep 10
mosquitto_pub -h 192.168.1.50 -u user1 -P password1 -r -t moisturesensor/ota_mode -n

