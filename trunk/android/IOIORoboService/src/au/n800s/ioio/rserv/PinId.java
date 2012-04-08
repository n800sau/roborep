package au.n800s.ioio.rserv;

public class PinId {

	//head ultrasonic servo (val:0 - 1000)
	static final int PWM_UHEAD = 13;

	//android phone turning servo
	static final int PWM_PHONE_TURN = 14;

	//android phone tilt servo
	static final int PWM_PHONE_TILT = 5;

	//arm lower joint turning servo
	static final int PWM_ARM_LOWER_TURN = 6;

	//arm lower joint tilt servo
	static final int PWM_ARM_LOWER_TILT = 7;

	//arm hand joint turning servo
	static final int PWM_ARM_HAND_TURN = 10;

	//arm hand joint tilt servo
	static final int PWM_ARM_HAND_TILT = 11;

	//arm hand grip servo
	static final int PWM_ARM_HAND_GRIP = 12;

	//uart to picaxe and usonic
	static final int UART_USONIC_RX = 27;
	static final int UART_USONIC_TX = 28;
}
