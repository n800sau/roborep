package au.n800s.ioio.rserv;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Intent;
import android.os.IBinder;

import android.os.Messenger;
import android.os.Message;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOService;

public class IOIORemoteService extends IOIOService {
	 /** For showing and hiding our notification. */
    NotificationManager mNM;

	 /** Keeps track of all current registered clients. */
	ArrayList<Messenger> mClients = new ArrayList<Messenger>();

	 /** Keeps track of all current registered clients. */
	ArrayList<Message> mActions = new ArrayList<Messenge>();

    /**
     * Handler of incoming messages from clients.
     */
    class IncomingHandler extends Handler {

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case MessageId.MSG_REGISTER_CLIENT:
                    mClients.add(msg.replyTo);
                    break;
                case MessageId.MSG_UNREGISTER_CLIENT:
                    mClients.remove(msg.replyTo);
                    break;
                case MessageId.MSG_SET_VALUE:
                    mValue = msg.arg1;
                    for (int i=mClients.size()-1; i>=0; i--) {
                        try {
                            mClients.get(i).send(Message.obtain(null, MessageId.MSG_SET_VALUE, mValue, 0));
                        } catch (RemoteException e) {
                            // The client is dead.  Remove it from the list;
                            // we are going through the list from back to front
                            // so this is safe to do inside the loop.
                            mClients.remove(i);
                        }
                    }
                    break;
                case MessageId.MSG_ACTION:
                    synchronized(mActions) {
						mActions.add(msg);
					}
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


	@Override
	protected IOIOLooper createIOIOLooper() {
		return new BaseIOIOLooper() {
			private DigitalOutput led_;

			@Override
			protected void setup() throws ConnectionLostException,
					InterruptedException {
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN);
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
				Message msg = null;
				synchronized(mActions) {
					if(!mActions.isEmpty()) {
						msg = mActions.get(0);
						mActions.remove(0);
					}
				}
				if( msg != null ) {
			        Toast.makeText(this, "Incoming " + msg.what, Toast.LENGTH_SHORT).show();
					msg.replyTo.send(Message.obtain(null, MessageId.MSG_ACTION, 0, 0));
				}
				led_.write(false);
				Thread.sleep(500);
				led_.write(true);
				Thread.sleep(500);
			}
		};
	}

	@Override
	public void onCreate() {
		super.onCreate();
		 mNM = (NotificationManager)getSystemService(NOTIFICATION_SERVICE);
	}

	@Override
	public void onDestroy() {
        // Cancel the persistent notification.
        mNM.cancel(R.string.remote_service_started);

        // Tell the user we stopped.
        Toast.makeText(this, R.string.remote_service_stopped, Toast.LENGTH_SHORT).show();

		super.onDestroy();
	}

	@Override
	public void onStart(Intent intent, int startId) {
		super.onStart(intent, startId);
		if (intent != null && intent.getAction() != null && intent.getAction().equals("stop")) {
			// User clicked the notification. Need to stop the service.
			mNM.cancel(0);
			stopSelf();
		} else {
			// Service starting. Create a notification.
			Notification notification = new Notification(R.drawable.ic_launcher, "IOIO service running", System.currentTimeMillis());
			notification.setLatestEventInfo(this, "IOIO Service", "Click to stop", PendingIntent.getService(this, 0, new Intent("stop", null, this, this.getClass()), 0));
			notification.flags |= Notification.FLAG_ONGOING_EVENT;
			mNM.notify(0, notification);
		}
	}

	@Override
	public IBinder onBind(Intent arg0) {
		return mMessenger.getBinder();
	}

}
