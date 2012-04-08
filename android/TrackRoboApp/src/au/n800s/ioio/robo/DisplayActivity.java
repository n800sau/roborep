package au.n800s.ioio.robo;

import android.app.Activity;
import android.content.Intent;

public class DisplayActivity extends Activity implements View.OnClickListener {

/** Messenger for communicating with service. */
Messenger mService = null;
/** Flag indicating whether we have called bind on the service. */
boolean mIsBound;
/** Some text view we are using to show state information. */
TextView mCallbackText;


	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		mCallbackText = ((TextView)findViewById(R.id.callbackText));
		findViewById(R.id.action).setOnClickListener(this);
		doBindService();
	}

    @Override
    public void onDestroy() {
		doUnbindService();
        super.onDestroy();
    }


    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.action: {
	            Message msg = Message.obtain(null, MessageId.MSG_SERVO, PinId.PWM_UHEAD, 1000);
	            msg.replyTo = mMessenger;
				mService.send(msg);
                break;
            }

            case R.id.sound: {
	            Message msg = Message.obtain(null, MessageId.MSG_HEAD_USONIC_DATA);
	            msg.replyTo = mMessenger;
				mService.send(msg);
                break;
            }
        }
    }


/**
 * Handler of incoming messages from service.
 */
class IncomingHandler extends Handler {
    @Override
    public void handleMessage(Message msg) {
        switch (msg.what) {
            case MessageId.MSG_SET_VALUE:
                mCallbackText.setText("Received from service: " + msg.arg1);
                break;
            case MessageId.MSG_ACTION:
                mCallbackText.setText("Action fulfilled");
                break;
            default:
                super.handleMessage(msg);
        }
    }
}

/**
 * Target we publish for clients to send messages to IncomingHandler.
 */
final Messenger mMessenger = new Messenger(new IncomingHandler());

/**
 * Class for interacting with the main interface of the service.
 */
private ServiceConnection mConnection = new ServiceConnection() {
    public void onServiceConnected(ComponentName className, IBinder service) {
        // This is called when the connection with the service has been
        // established, giving us the service object we can use to
        // interact with the service.  We are communicating with our
        // service through an IDL interface, so get a client-side
        // representation of that from the raw service object.
        mService = new Messenger(service);
        mCallbackText.setText("Attached.");

        // We want to monitor the service for as long as we are
        // connected to it.
        try {
            Message msg = Message.obtain(null, MessageId.MSG_REGISTER_CLIENT);
            msg.replyTo = mMessenger;
            mService.send(msg);

            // Give it some value as an example.
            msg = Message.obtain(null,
                    MessageId.MSG_SET_VALUE, this.hashCode(), 0);
            mService.send(msg);
        } catch (RemoteException e) {
            // In this case the service has crashed before we could even
            // do anything with it; we can count on soon being
            // disconnected (and then reconnected if it can be restarted)
            // so there is no need to do anything here.
        }

        // As part of the sample, tell the user what happened.
        Toast.makeText(Binding.this, R.string.remote_service_connected,
                Toast.LENGTH_SHORT).show();
    }

    public void onServiceDisconnected(ComponentName className) {
        // This is called when the connection with the service has been
        // unexpectedly disconnected -- that is, its process crashed.
        mService = null;
        mCallbackText.setText("Disconnected.");

        // As part of the sample, tell the user what happened.
        Toast.makeText(Binding.this, R.string.remote_service_disconnected,
                Toast.LENGTH_SHORT).show();
    }
};

void doBindService() {
    // Establish a connection with the service.  We use an explicit
    // class name because there is no reason to be able to let other
    // applications replace our component.
    bindService(new Intent("au.n800s.ioio.rserv.IOIORemoteService"), mConnection, Context.BIND_AUTO_CREATE);
    mIsBound = true;
    mCallbackText.setText("Binding.");
}

void doUnbindService() {
    if (mIsBound) {
        // If we have received the service, and hence registered with
        // it, then now is the time to unregister.
        if (mService != null) {
            try {
                Message msg = Message.obtain(null,
                        MessageId.MSG_UNREGISTER_CLIENT);
                msg.replyTo = mMessenger;
                mService.send(msg);
            } catch (RemoteException e) {
                // There is nothing special we need to do if the service
                // has crashed.
            }
        }

        // Detach our existing connection.
        unbindService(mConnection);
        mIsBound = false;
        mCallbackText.setText("Unbinding.");
    }
}

}