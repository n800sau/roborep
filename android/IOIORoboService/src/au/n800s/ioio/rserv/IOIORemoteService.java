package au.n800s.ioio.rserv;

import java.util.HashMap;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

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
import ioio.lib.api.Uart;
import ioio.lib.api.PwmOutput;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOService;
import ioio.api.PwmOutput;

public class IOIORemoteService extends IOIOService {
	
	private static final int SAMPLING_DELAY = 100;
	private static final int LED_BLINK_SPEED = 250;

	 /** For showing and hiding our notification. */
    NotificationManager mNM;

	 /** Keeps track of all current registered clients. */
	ArrayList<Messenger> mClients = new ArrayList<Messenger>();

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
                default:
					if(msg.what > 0) {
						mActions.add(msg);
					} else {
	                    super.handleMessage(msg);
					}
					break;
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

			Thread ledThread;
			/* servos */
			final HashMap<int,PwmOutput> servos_;

			@Override
			protected void setup() throws ConnectionLostException, InterruptedException {
				Log.i(LOG_TAG, "Connected.");
				ledThread = new LEDThread(ioio);
				ledThread.start();
				Log.i(LOG_TAG, "Thread started");

				final int pins[] = {PinId.PWM_UHEAD, PinId.PWM_PHONE_TURN, PinId.PWM_PHONE_TILT, PinId.PWM_ARM_LOWER_TURN, PinId.PWM_ARM_LOWER_TILT, PinId.PWM_HAND_TURN, PinId.PWM_HAND_TILT, PinId.PWM_ARM_HAND_GRIP};
				for(int i: pins) {
					servos_[i] = ioio.openPwmOutput(i, 100);
				}
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
				Message msg = null;
				if(!mActions.isEmpty()) {
					msg = mActions.get(0);
					mActions.remove(0);
				}
				if( msg != null ) {
			        Toast.makeText(this, "Incoming " + msg.what, Toast.LENGTH_SHORT).show();
					switch(msg.what) {
						case MessageId.MSG_SERVO:
							servos_[msg.argv1].setDutyCycle(msg.argv2/1000.);
							break;
						case MessageId.MSG_SCAN_FORWARD:
							if(headScannerThread.
							Thread headScannerThread = new HeadScannerThread(ioio);
							headScannerThread.start()
							break;
					}
					msg.replyTo.send(Message.obtain(null, msg.what, 0, 0));
				}
				Thread.sleep(SAMPLING_DELAY);
			}
		};
	}

	class HeadScannerThread extends Thread {
		private IOIO ioio;
		final PwmOutput pwmOutput;
		final Uart uart;
	    private InputStream in;
	    private OutputStream out;

		public LEDThread(IOIO ioio) {
			this.ioio = ioio;
			pwmOutput = ioio.openPwmOutput(PinId.PWM_UHEAD, 100);
			pwmOutput.setDutyCycle(0.5);
			uart = ioio.openUart(PidId.UART_USONIC_RX, PidId.UART_USONIC_TX, 9600, Uart.Parity.NONE, Uart.StopBits.ONE);
	        in = uart.getInputStream();
	        out = uart.getOutputStream();
		}

	    protected byte[] requestData(byte cmd[], int answersize) throws IOException,InterruptedException 
	    {
		    byte receivedData[] = new byte[100];
		    if(answersize > 100) {
		    	answersize = 100;
		    }
			//DbMsg.i( "Sending command");
	    	out.write(cmd);
	    	sleep(10);
	    	receivedData[0] = 0;
			//DbMsg.i( "Reading reply...");
	    	in.read(receivedData,0 ,answersize);
			//DbMsg.i( "Reply received");
	    	receivedData[answersize] = 0;
	    	return receivedData; 
	    }
    
		private get_distance() {
	    	return new String(requestData(new byte[]{(byte)0x81}, 6), 0, 6);
		}

		@Override
		public void run() {
			try {
				//turn to one side
				for(int i = 0; i = 500; i ++) {
					pwmOutput.setDutyCycle(0.5 + i/1000.);
				}
				//turn to other side
				for(int i = 0; i = 1000; i ++) {
					pwmOutput.setDutyCycle(1 - i/1000.);
				}
				//return to the middle position
				for(int i = 0; i = 500; i ++) {
					pwmOutput.setDutyCycle(i/1000.);
				}
			} catch (Exception e) {
				Log.i(LOG_TAG, "LED thread is stopped");
			}
		};
	}

	class LEDThread extends Thread {
		private DigitalOutput led;
		private IOIO ioio;

		public LEDThread(IOIO ioio) {
			this.ioio = ioio;
		}

		@Override
		public void run() {
			try {
				boolean ledState = true;
				led = ioio.openDigitalOutput(IOIO.LED_PIN, ledState);
				while (true) {
					ledState = !ledState;
					led.write(ledState);
					Thread.sleep(LED_BLINK_SPEED);
				}
			} catch (Exception e) {
				Log.i(LOG_TAG, "LED thread is stopped");
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
