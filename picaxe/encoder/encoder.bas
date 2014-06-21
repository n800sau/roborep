symbol right_encoder = c.1
symbol serialpin = c.4
symbol right_level = w0
symbol right_min = w1
symbol right_max = w2
symbol tmp1 = w3

right_min = 30000
right_max = 0


main:

readadc10 right_encoder, right_level

if right_level < right_min then
	right_min = right_level
endif

if right_level > right_max then
	right_max = right_level
endif

tmp1 = right_max - right_min
tmp1 = tmp1 / 2 + right_min

if right_level < tmp1 then
	right_level = 0
else
	right_level = 1
endif

' output for inverted picaxe serial
sertxd("Right:",#right_level," , ", #tmp1, 13,10)

' output for standard serial
serout serialpin, T4800_4, ("Right",#right_level,13,10)

goto main 'and around for
