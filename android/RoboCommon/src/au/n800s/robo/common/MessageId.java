package au.n800s.robo.common;

public class MessageId {
	/**
	 * Command to the service to register a client, receiving callbacks
	 * from the service.  The Message's replyTo field must be a Messenger of
	 * the client where callbacks should be sent.
	 */
	static public final int MSG_REGISTER_CLIENT = 10;

	/**
	 * Command to the service to unregister a client, ot stop receiving callbacks
	 * from the service.  The Message's replyTo field must be a Messenger of
	 * the client as previously given with MSG_REGISTER_CLIENT.
	 */
	static public final int MSG_UNREGISTER_CLIENT = 20;

	/**
	 * Command to service to set a new value.  This can be sent to the
	 * service to supply a new value, and will be sent by the service to
	 * any registered clients with the new value.
	 */
	static public final int MSG_SET_VALUE = 30;

	/* Robo Servo message */
	static public final int MSG_SERVO = 40;

	/* Robo Sound message */
	static public final int MSG_SCAN_FORWARD = 50;

	/* Robo Head Usonic Data */
	static public final int MSG_HEAD_USONIC_DATA = 60;

	/* Robo Go Forward */
	static public final int MSG_MOVE_STRAIGHT = 70;

	/* Robo Turn Right */
	static public final int MSG_TURN_RIGHT = 90;

	/* Robo Turn Left */
	static public final int MSG_TURN_LEFT = 110;

	/* Robo Full Stop */
	static public final int MSG_FULL_STOP = 120;

	/* Robo Battery */
	static public final int MSG_BATTERY = 130;

	/* Robo Charger */
	static public final int MSG_CHARGER = 140;

	/* Robo Base LED on/off */
	static public final int MSG_BASE_LED = 150;

}
