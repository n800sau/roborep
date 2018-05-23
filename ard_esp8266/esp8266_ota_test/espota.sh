./espota.py --ip=balancebot.local -f .pioenvs/esp01/firmware.bin -a Zagruzka -r -d &>espota.log
#./espota.py --ip=balancebot.local -f data.spiff -a Zagruzka -s -d &>espota.log
#./espota.py --ip=myesp8266.local -f .pioenvs/esp01/firmware.bin -a Zagruzka -r -d &>espota.log
#./espota.py --ip=myesp8266.local -f data.spiff -a Zagruzka -s &>espota.log
./espota.py --ip=ESP8266-OTA-aac4d3.local -f data.spiff -s &>espota.log
echo $?
