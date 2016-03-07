#!/usr/bin/env python

import pywapi


# e.g see https://weather.com/weather/today/l/ASNS0391:1:AS
codes = {
	'Sydney': 'ASXX0112',
	'Kirrawee': 'ASXX9462:1:AS',
	'Pyrmont': 'ASNS0391:1:AS',
	'Bendalong': 'ASXX1792:1:AS',
}

def get_weather_list(location):

	code = codes[location]
	data = pywapi.get_weather_from_weather_com(code)
	current = data['current_conditions']
	nextday = data['forecasts'][1]
	rs = ['%s.' % data['location']['name'],
		'Temperature %s.' % current['temperature'],
		'Humidity %s.' % current['humidity'],
		'Wind %d meter per second' % (int(current['wind']['speed']) * 1000 // 3600),
		'Tomorrow %s.' % nextday['day']['text'],
		'Temperature %s.' % nextday['high'],
		'Day wind %d meter per second ' % (int(nextday['day']['wind']['speed']) * 1000 // 3600),
	]
	return rs

if __name__ == '__main__':

	wt = get_weather_list('Kirrawee')
	print '\n'.join(wt)
