package au.n800s.ioio.rserv;

import java.util.ArrayList;
import java.util.HashMap;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Intent;
import android.os.IBinder;

import android.os.Handler;
import android.os.Messenger;
import android.os.Message;
import android.os.RemoteException;
import android.widget.Toast;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.api.Uart;
import ioio.lib.api.PwmOutput;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOService;
import ioio.lib.api.PwmOutput;

public class IOIORemoteService extends IOIOService {
	
	private static final int SAMPLING_DELAY = 100;
	private static final int LED_BLINK_SPEED = 250;

	 /** For showing and hiding our notification. */
    NotificationManager mNM;

	 /** Keeps track of all current registered clients. */
	ArrayList<Messenger> mClients = new ArrayList<Messenger>();

	ArrayList<Message> mActions = new ArrayList<Message>();

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
                    int mValue = msg.arg1;
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
			final HashMap<Integer,PwmOutput> servos_ = new HashMap<Integer,PwmOutput>();

			final int pins[] = {PinId.PWM_UHEAD, PinId.PWM_PHONE_TURN, PinId.PWM_PHONE_TILT, PinId.PWM_ARM_LOWER_TURN, PinId.PWM_ARM_LOWER_TILT, PinId.PWM_ARM_HAND_TURN, PinId.PWM_ARM_HAND_TILT, PinId.PWM_ARM_HAND_GRIP};

			@Override
			protected void setup() throws ConnectionLostException, InterruptedException {
				DbMsg.i("Connected.");
				ledThread = new LEDThread(ioio_);
				ledThread.start();
				DbMsg.i("Thread started");
//				for(int i: pins) {
//					DbMsg.i("Pin Pwm init=" + i);
//					servos_.put(i, ioio_.openPwmOutput(i, 100));
//				}
				Thread headScannerThread = new HeadScannerThread(ioio_);
				headScannerThread.start();
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
				Message msg = null;
				if(!mActions.isEmpty()) {
					msg = mActions.get(0);
					mActions.remove(0);
				}
				if( msg != null ) {
			        Toast.makeText(IOIORemoteService.this, "Incoming " + msg.what, Toast.LENGTH_SHORT).show();
					switch(msg.what) {
						case MessageId.MSG_SERVO:
							servos_.get(msg.arg1).setDutyCycle(msg.arg2/1000f);
							break;
						case MessageId.MSG_SCAN_FORWARD:
							Thread headScannerThread = new HeadScannerThread(ioio_);
							headScannerThread.start();
							break;
					}
					try {
						msg.replyTo.send(Message.obtain(null, msg.what, 0, 0));
					} catch(RemoteException e) {
						DbMsg.e("Reply error:", e);						
					}
				}
				Thread.sleep(SAMPLING_DELAY);
			}
		};
	}

	class HeadScannerThread extends Thread {
		private IOIO ioio;
		private PwmOutput pwmOutput;
		private Uart uart;
	    private InputStream in;
	    private OutputStream out;

		public HeadScannerThread(IOIO ioio) throws ConnectionLostException  {
			this.ioio = ioio;
			pwmOutput = this.ioio.openPwmOutput(PinId.PWM_UHEAD, 100);
			pwmOutput.setDutyCycle(0.5f);
			DbMsg.i("centered");
//			uart = ioio.openUart(PinId.UART_USONIC_RX, PinId.UART_USONIC_TX, 9600, Uart.Parity.NONE, Uart.StopBits.ONE);
//	        in = uart.getInputStream();
//	        out = uart.getOutputStream();
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
    
		private String get_distance() throws IOException, InterruptedException {
	    	return new String(requestData(new byte[]{(byte)0x81}, 6), 0, 6);
		}

		@Override
		public void run() {
			try {
				while(true) {
					DbMsg.i("head start");
					pwmOutput.setPulseWidth(1500);
					//turn to one side
					for(int i = 1500; i < 2500; i += 10) {
						pwmOutput.setPulseWidth(i);
						DbMsg.i("i="+i);
						Thread.sleep(100);
					}
					Thread.sleep(1000);
					//turn to other side
					for(int i = 2500; i > 500; i -= 10) {
						pwmOutput.setPulseWidth(i);
						DbMsg.i("i="+i);
						Thread.sleep(100);
					}
					Thread.sleep(1000);
					//return to the middle position
					for(int i = 500; i < 1500; i += 10) {
						pwmOutput.setPulseWidth(i);
						DbMsg.i("i="+i);
						Thread.sleep(100);
					}
					Thread.sleep(1000);
					DbMsg.i("head end");
				}
			} catch (Exception e) {
				DbMsg.e("Head Scanner stopped", e);
			}
			DbMsg.e("Head Scanner stopped Ok");
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
				DbMsg.e("LED thread is stopped", e);
			}
			}
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
			notification.setLatestEventInfo(this, "IOIO Robotarr Service", "Click to stop", PendingIntent.getService(this, 0, new Intent("stop", null, this, this.getClass()), 0));
			notification.flags |= Notification.FLAG_ONGOING_EVENT;
			mNM.notify(0, notification);
		}
	}

	@Override
	public IBinder onBind(Intent arg0) {
		return mMessenger.getBinder();
	}

}
