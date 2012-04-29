package au.n800s.robo.model.track;

public class PinId {

	//head ultrasonic servo (val:0 - 1000)
	static public final int PWM_UHEAD = 13;

	//android phone turning servo
	static public final int PWM_PHONE_TURN = 14;

	//android phone tilt servo
	static public final int PWM_PHONE_TILT = 5;

	//arm lower joint turning servo
	static public final int PWM_ARM_LOWER_TURN = 6;

	//arm lower joint tilt servo
	static public final int PWM_ARM_LOWER_TILT = 7;

	//arm hand joint turning servo
	static public final int PWM_ARM_HAND_TURN = 10;

	//arm hand joint tilt servo
	static public final int PWM_ARM_HAND_TILT = 11;

	//arm hand grip servo
	static public final int PWM_ARM_HAND_GRIP = 12;

	//uart to picaxe and usonic
	static public final int UART_USONIC_RX = 27;
	static public final int UART_USONIC_TX = 28;

	//i2c to base pro interface index ioio.lib.impl.Constants - 47,48 pins
	static public final int BASE_I2C_INDEX = 1;
}
