package au.n800s.robo.common;

public class BaseProCommands {

	//address of base pro
	static public final int I2C_Addr=19;

	//left motor command
	static public final char CMD_LEFT = 'l';


	//right motor command
	static public final char CMD_RIGHT = 'r';

	//both motors command
	static public final char CMD_BOTH = 'b';

	//stop both motors command
	static public final char CMD_STOP = 's';

	//get battery voltage command
	static public final char CMD_BATTERY = 'v';

	//get charger voltage command
	static public final char CMD_CHARGER = 'c';

	//turn on/off the led command
	static public final char CMD_LED = 'd';

}
