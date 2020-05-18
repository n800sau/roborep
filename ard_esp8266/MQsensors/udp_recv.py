#!/usr/bin/env python

import sys
import socket
import struct
import json
import time
import traceback
import numbers
import redis
from math import log10
from datetime import datetime

import paho.mqtt.client as mqtt

MQ_CLIENT_NAME = 'mqsensors'
MQTT_BROKER_HOST = 'localhost'
MQTT_BROKER_PORT = 1883

#device setup
j = """
{
	"deviceInfo": {
		"status": "good",
		"color": "#4D90FE",
		"endPoints": {
			"mq2": {
				"units": "voltage",
				"values": {
					"value": 0
				},
				"card-type": "crouton-simple-text",
				"title": "MQ2 V"
			}
		},
		"description": "MQ devices"
	}
}

"""

device = json.loads(j)
device["deviceInfo"]["name"] = MQ_CLIENT_NAME
deviceJson = json.dumps(device)


MCAST_GRP = '239.0.0.57'
MCAST_PORT = 8989

REDIS_KEY = 'mq2_list'
MAX_ITEMS = 100

def dbprint(text):
#	print('[%s]:%s' % (datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text), file=sys.__stderr__)
	print >>sys.__stderr__, '[%s]:%s' % (datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

def make_sock():

	i = 1
	while True:

		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
							 # to MCAST_GRP, not all groups on MCAST_PORT
			mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

			sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
			break

		except:
			traceback.print_exc()
			time.sleep(5)
			dbprint('Attempt %d to bind' % i)
			i += 1

	return sock


#callback when we recieve a connack
def on_mqconnect(client, userdata, flags, rc):
	print("Connected with result code " + str(rc))

#callback when we receive a published message from the server
def on_mqmessage(client, userdata, msg):
	print(msg.topic + ": " + str(msg.payload))
	box = msg.topic.split("/")[1]
	name = msg.topic.split("/")[2]
	address = msg.topic.split("/")[3]
	if box == "inbox" and str(msg.payload) == "get" and address == "deviceInfo":
		mqclient.publish("/outbox/" + MQ_CLIENT_NAME + "/deviceInfo", deviceJson) #for autoreconnect

def on_mqdisconnect(client, userdata, rc):
	if rc != 0:
		print("Broker disconnection")
	time.sleep(10)
	mqclient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)

def fill_wells(r, data):
	ts_format = '%Y-%m-%d %H:%M:%S'
	ts = data['ts']
	sensor_id = data['sensor_id']
	for k,v in data.items():
		if k not in ('ts', 'sensor_id') and isinstance(v, numbers.Number):
			r_dt = datetime.fromtimestamp(ts)
			r_name = 'last.' + sensor_id + '.' + k
			r.set(r_name, json.dumps({'v': v, 'ts': ts}))
			for w_name, r_w in [
						['monthly', r_dt.replace(day=1, hour=0, microsecond=0, second=0, minute=0).strftime(ts_format)],
						['daily', r_dt.replace(hour=0, microsecond=0, second=0, minute=0).strftime(ts_format)],
						['hourly', r_dt.replace(microsecond=0, second=0, minute=0).strftime(ts_format)],
					]:
				r_name = 'avg.' + sensor_id + '.' + w_name + '.' + k
				w_d = r.hget(r_name, r_w)
				if not w_d:
					w_d = {'v': 0, 'count': 0}
				else:
					w_d = json.loads(w_d)
				w_d['count'] += 1
				w_d['v'] += (v - w_d['v']) / w_d['count']
				r.hset(r_name, r_w, json.dumps(w_d))

def main():

	sock = make_sock()

	r = redis.Redis()

	mqclient = mqtt.Client(MQ_CLIENT_NAME)
	mqclient.on_connect = on_mqconnect
	mqclient.on_message = on_mqmessage
	mqclient.on_disconnect = on_mqdisconnect
	mqclient.username_pw_set("","")
	mqclient.will_set('/outbox/' + MQ_CLIENT_NAME + '/lwt', '{"message":"I am gone for good"}', 0, False)
	mqclient.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
	mqclient.subscribe("/inbox/" + MQ_CLIENT_NAME + "/deviceInfo")
	mqclient.publish("/outbox/" + MQ_CLIENT_NAME + "/deviceInfo", deviceJson) #for autoreconnect
	for key in device["deviceInfo"]["endPoints"]:
		#print key
		mqclient.subscribe("/inbox/" + MQ_CLIENT_NAME + "/"+str(key))

	mqclient.loop_start()
	while True:
		try:
#			dbprint('Waiting...')
			data,address = sock.recvfrom(1000)
			try:
				jdata = json.loads(data.decode())
				if 'sensor_id' in jdata:
					if 'ts' not in data:
						data['ts'] = int(time.time())
					dbprint('%g at %s from %s (%s)' % (jdata['rawval'], jdata['ts'], jdata['sensor_id'], ':'.join([str(v) for v in address])))
					r.rpush(REDIS_KEY, json.dumps(jdata))
#					mqclient.publish("/outbox/" + MQ_CLIENT_NAME + "/mq2", json.dumps(jdata))
					mqclient.publish("/hass/sensor/mq2/state", json.dumps(jdata))
					fill_wells(r, jdata)
#					mqclient.publish("/outbox/" + MQ_CLIENT_NAME + "/mq2", jdata['rawval'])
					while r.llen(REDIS_KEY) > MAX_ITEMS:
						r.lpop(REDIS_KEY)
			except ValueError as e:
				dbprint('%s: %s' % (e, data))
				continue
		except Exception as e:
			if isinstance(e, KeyboardInterrupt):
				raise
			traceback.print_exc()

# define the load resistance on the board, in kilo ohms
RL_VALUE_MQ2 = 47 + 10
# RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO which is derived from the chart in datasheet
RO_CLEAN_AIR_FACTOR_MQ2 = 9.577

# define how many samples you are going to take in the calibration phase
CALIBARAION_SAMPLE_TIMES = 50

# define the time interal(in milisecond) between each samples in the cablibration phase
CALIBRATION_SAMPLE_INTERVAL = 500

# define how many samples you are going to take in normal operation
READ_SAMPLE_INTERVAL = 50
# define the time interal(in milisecond) between each samples
READ_SAMPLE_TIMES = 5

GAS_HYDROGEN = 0
GAS_LPG = 1
GAS_METHANE = 2
GAS_CARBON_MONOXIDE = 3
GAS_ALCOHOL = 4
GAS_SMOKE = 5
GAS_PROPANE = 6

accuracy = 0 # for linearcurves
#accuracy = 1 # for nonlinearcurves, un comment this line and comment the above line if calculations are to be done using non linear curve equations

def MQResistanceCalculation(voltage):
	return float(RL_VALUE_MQ2) * (5 - voltage) / voltage

def MQCalibration(mq_pin):
	RS_AIR_val = 0
	for i in range(CALIBARAION_SAMPLE_TIMES): # take multiple samples
		RS_AIR_val += MQResistanceCalculation()
		time.sleep(CALIBRATION_SAMPLE_INTERVAL)
	RS_AIR_val = RS_AIR_val / CALIBARAION_SAMPLE_TIMES # calculate the average value
	# RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro
	# according to the chart in the datasheet
	r0 = RS_AIR_val / RO_CLEAN_AIR_FACTOR_MQ2
	return r0;


def MQGetGasPercentage(rs_ro_ratio, gas_id):
	if accuracy == 0:
		if gas_id == GAS_HYDROGEN:
			return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)))
		elif gas_id == GAS_LPG:
			return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)))
		elif gas_id == GAS_METHANE:
			return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)))
		elif gas_id == GAS_CARBON_MONOXIDE:
			return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)))
		elif gas_id == GAS_ALCOHOL:
			return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)))
		elif gas_id == GAS_SMOKE:
			return (pow(10,((-2.331*(log10(rs_ro_ratio))) + 3.596)))
		elif gas_id == GAS_PROPANE:
			return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)))
	elif accuracy == 1:
		if gas_id == GAS_HYDROGEN:
			return (pow(10,((-2.109*(log10(rs_ro_ratio))) + 2.983)))
		elif gas_id == GAS_LPG:
			return (pow(10,((-2.123*(log10(rs_ro_ratio))) + 2.758)))
		elif gas_id == GAS_METHANE:
			return (pow(10,((-2.622*(log10(rs_ro_ratio))) + 3.635)))
		elif gas_id == GAS_CARBON_MONOXIDE:
			return (pow(10,((-2.955*(log10(rs_ro_ratio))) + 4.457)))
		elif gas_id == GAS_ALCOHOL:
			return (pow(10,((-2.692*(log10(rs_ro_ratio))) + 3.545)))
		elif gas_id == GAS_SMOKE:
			return (pow(10,(-0.976*pow((log10(rs_ro_ratio)), 2) - 2.018*(log10(rs_ro_ratio)) + 3.617)))
		elif gas_id == GAS_PROPANE:
			return (pow(10,((-2.174*(log10(rs_ro_ratio))) + 2.799)))
	return 0

if __name__ == '__main__':

	main()
