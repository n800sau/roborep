package au.n800s.robo.model.track;

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
import android.os.RemoteException;
import android.widget.Toast;

import au.n800s.track.common.DbMsg;
import au.n800s.track.common.IRobo;
import au.n800s.track.common.IRoboCallback;

import au.n800s.track.common.PinId;
import au.n800s.track.common.BaseProCommands;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.TwiMaster;
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

	private final IRobo.Stub mBinder = new IRobo.Stub() {

		public String getName()
		{
			return "Track";
		}

		public String getVersion()
		{
			return "0.0";
		}

		public Float getBattery()
		{
			DbMsg.i("battery voltage requested");
			request = ("" + BaseProCommands.CMD_BATTERY).getBytes();
			response = new byte[50];
			i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
			return new Float(new String(response));
		}

	};


	private BaseIOIOLooper mLooper = new BaseIOIOLooper() {
			Thread ledThread;
			Thread headScannerThread;
			/* servos */
			final HashMap<Integer,PwmOutput> servos_ = new HashMap<Integer,PwmOutput>();

			final int pins[] = {PinId.PWM_UHEAD, PinId.PWM_PHONE_TURN, PinId.PWM_PHONE_TILT, PinId.PWM_ARM_LOWER_TURN, PinId.PWM_ARM_LOWER_TILT, PinId.PWM_ARM_HAND_TURN, PinId.PWM_ARM_HAND_TILT, PinId.PWM_ARM_HAND_GRIP};

			private TwiMaster i2cBaseMaster;

			@Override
			protected void setup() throws ConnectionLostException, InterruptedException {
				DbMsg.i("Connected.");
				ledThread = new LEDThread(ioio_);
				ledThread.start();
				DbMsg.i("Thread started");
				for(int i: pins) {
					DbMsg.i("Pin Pwm init=" + i);
					servos_.put(i, ioio_.openPwmOutput(i, 100));
				}
				headScannerThread = new HeadScannerThread(ioio_, servos_.get(PinId.PWM_UHEAD));
				i2cBaseMaster = ioio_.openTwiMaster(PinId.BASE_I2C_INDEX, TwiMaster.Rate.RATE_100KHz, false);
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
				Message msg = null;
				Message reply = null;
				byte[] request;
				byte[] response;
				String speed;
				if(!mActions.isEmpty()) {
					msg = mActions.get(0);
					DbMsg.i("msg found what="+msg);
				}
				if( msg != null ) {
					DbMsg.i("msg found");
					switch(msg.what) {
					case MessageId.MSG_SERVO:
						DbMsg.i("servo requested"+msg.arg1+","+msg.arg2);
						servos_.get(msg.arg1).setPulseWidth(msg.arg1);
						break;
					case MessageId.MSG_SCAN_FORWARD:
						DbMsg.i("scan forward requested");
						headScannerThread.start();
						break;
					case MessageId.MSG_MOVE_STRAIGHT:
						DbMsg.i("move straight requested");
						String speedleft = Integer.toString(msg.arg1);
						String speedright = Integer.toString(msg.arg2);
						request = (BaseProCommands.CMD_BOTH + speedleft + ';' + speedright).getBytes();
						response = new byte[0];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						break;
					case MessageId.MSG_TURN_RIGHT:
						DbMsg.i("turn right requested");
						speed = Integer.toString(msg.arg1);
						request = (BaseProCommands.CMD_RIGHT + speed).getBytes();
						response = new byte[0];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						break;
					case MessageId.MSG_TURN_LEFT:
						DbMsg.i("turn left requested");
						speed = Integer.toString(msg.arg1);
						request = (BaseProCommands.CMD_LEFT + speed).getBytes();
						response = new byte[0];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						break;
					case MessageId.MSG_BATTERY:
						DbMsg.i("battery voltage requested");
						request = ("" + BaseProCommands.CMD_BATTERY).getBytes();
						response = new byte[50];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						reply = Message.obtain(null, msg.what, new Float(new String(response)));
						break;
					case MessageId.MSG_CHARGER:
						DbMsg.i("charger voltage requested");
						request = ("" + BaseProCommands.CMD_CHARGER).getBytes();
						response = new byte[50];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						reply = Message.obtain(null, msg.what, new Float(new String(response)));
						break;
					case MessageId.MSG_BASE_LED:
						DbMsg.i("base led command requested");
						String on = Boolean.toString(msg.arg1 != 0);
						request = (BaseProCommands.CMD_LED + on).getBytes();
						response = new byte[0];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						break;
					case MessageId.MSG_FULL_STOP:
						DbMsg.i("full stop requested");
						request = ("" + BaseProCommands.CMD_STOP).getBytes();
						response = new byte[0];
						i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
						break;
					}
					DbMsg.i("after");
					if(reply == null) {
						reply = Message.obtain(null, msg.what, 0, 0);
					}
					try {
						msg.replyTo.send(reply);
					} catch(RemoteException e) {
						DbMsg.e("Reply error:", e);
					}
					mActions.remove(0);
				}
				Thread.sleep(SAMPLING_DELAY);
			}
		};


	@Override
	protected IOIOLooper createIOIOLooper() {
		return mLooper;
	}

	class HeadScannerThread extends Thread {
		private IOIO ioio;
		private PwmOutput pwmOutput;
		private Uart uart;
		private InputStream in;
		private OutputStream out;

		public HeadScannerThread(IOIO ioio, PwmOutput servo) throws ConnectionLostException  {
			this.ioio = ioio;
			pwmOutput = servo;
			pwmOutput.setPulseWidth(1500);
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
				DbMsg.i("head start");
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
			} catch (Exception e) {
				DbMsg.e("Head Scanner stopped", e);
			}
		};
	}


	@Override
	public void onCreate() {
		super.onCreate();
		mNM = (NotificationManager)getSystemService(NOTIFICATION_SERVICE);
		// Tell the user we started.
		Toast.makeText(this, R.string.remote_service_started, Toast.LENGTH_SHORT).show();
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
			Notification notification = new Notification(R.drawable.ic_launcher, "Track service running", System.currentTimeMillis());
			notification.setLatestEventInfo(this, "Track Robotarr Service", "Click to stop", PendingIntent.getService(this, 0, new Intent("stop", null, this, this.getClass()), 0));
			notification.flags |= Notification.FLAG_ONGOING_EVENT;
			mNM.notify(0, notification);
		}
	}

	@Override
	public IBinder onBind(Intent arg0) {
		DbMsg.i("IBind");
		return mBinder;
	}

}
