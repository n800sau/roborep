#picaxe 20m2

symbol RESET_PIN = B.1
symbol STM_BOOT0_PIN = C.2
symbol ESP_1_PIN = C.3

symbol PROG_LED_PIN = B.2
symbol RESET_LED_PIN = B.3

symbol prog_btn = C.5
symbol reset_btn = C.6

	high PROG_LED_PIN
	high RESET_LED_PIN
	high RESET_PIN
	low STM_BOOT0_PIN
	high ESP_1_PIN
main:
	button reset_btn,1,200,100,b2,1,reset_pushed
	button prog_btn,1,200,100,b2,1,prog_pushed
	sertxd ("PASS")
	pause 10
reset_pushed:
	low RESET_LED_PIN
	low RESET_PIN
	pause 10
	high RESET_PIN
	sertxd ("RESET PUSH")
	high RESET_LED_PIN
	goto main
prog_pushed:
	low PROG_LED_PIN
	high STM_BOOT0_PIN
	low ESP_1_PIN
	pause 50
	low RESET_PIN
	pause 50
	high RESET_PIN
	pause 50
	low STM_BOOT0_PIN
	high ESP_1_PIN
	sertxd ("PROG PUSH")
	high PROG_LED_PIN
	goto main
