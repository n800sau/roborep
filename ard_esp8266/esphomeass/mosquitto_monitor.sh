#mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t '#'
#mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t '#'|grep moisture
#mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t 'moisturesensor/ota_mode'
#mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t 'moisturesensor/sensor/moisture/state'
mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t '#'|grep 9c9c1f4749aa303030
#mosquitto_sub -v -h localhost -p 1883 -u user1 -P password1 -t '#'|grep moisture_ra
