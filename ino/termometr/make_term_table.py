import math


Vcc = 3.3
Vref = 1.1
T_0 = 273.15;
T_25 = T_0 + 25;

#beta;
#R_25;
#Rs;

ntc_list = [
	{
		'beta': 3950,
		'R_25': 100000,
		'Rs': 470000,
		'name': 'ntc_3950_100',
	},
	{
		'beta': 3435,
		'R_25': 10000,
		'Rs': 47000,
		'name': '3435_10',
	},
]

for t in ntc_list:

	print('*' * 80)
	print(t['name'])
	last_temp = 0
	i = 0
	for tv in range(1, 1024):

		v = Vref * tv/1024.
		r = v/((Vcc-v)/t['Rs'])
#		print('r=', int(r))
		temp = int(round(1 / ((math.log(r / t['R_25']) / t['beta']) + 1/T_25) - T_0))
		if last_temp != temp and temp >= 5 and temp <= 150:
			print(tv, temp)
			last_temp = temp
			i += 1
	print('Count:', i)

