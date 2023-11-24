#!/bin/sh

HOST=192.168.1.87

curl http://${HOST}/digital/6
curl http://${HOST}/led?params=1
curl http://${HOST}/digital/4
curl http://${HOST}/led?params=0
curl http://${HOST}/digital/4
