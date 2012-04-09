package au.n800s.ioio.robo;

public class MessageId {
    /**
     * Command to the service to register a client, receiving callbacks
     * from the service.  The Message's replyTo field must be a Messenger of
     * the client where callbacks should be sent.
     */
    static final int MSG_REGISTER_CLIENT = 1;

    /**
     * Command to the service to unregister a client, ot stop receiving callbacks
     * from the service.  The Message's replyTo field must be a Messenger of
     * the client as previously given with MSG_REGISTER_CLIENT.
     */
    static final int MSG_UNREGISTER_CLIENT = 2;

    /**
     * Command to service to set a new value.  This can be sent to the
     * service to supply a new value, and will be sent by the service to
     * any registered clients with the new value.
     */
    static final int MSG_SET_VALUE = 3;

	/* Robo Servo message */
	static final int MSG_SERVO = 4;

	/* Robo Sound message */
	static final int MSG_SCAN_FORWARD = 5;
	
	/* Robo Head Usonic Data */
	static final int MSG_HEAD_USONIC_DATA = 6;

}
