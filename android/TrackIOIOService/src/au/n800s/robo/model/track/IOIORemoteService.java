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
import au.n800s.robo.common.BaseProCommands;
import au.n800s.robo.common.DbMsg;
import au.n800s.robo.common.IRobo;

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

	private static final int SAMPLING_DELAY = 1000;

	/** For showing and hiding our notification. */
	NotificationManager mNM;

	private BaseIOIOLooper mLooper = new BaseIOIOLooper() {
			Thread ledThread;
			Thread headScannerThread;

			/* servo */
			final HashMap<Integer,PwmOutput> servos_ = new HashMap<Integer,PwmOutput>();

			final int pins[] = {PinId.PWM_UHEAD, PinId.PWM_PHONE_TURN, PinId.PWM_PHONE_TILT, PinId.PWM_ARM_LOWER_TURN, PinId.PWM_ARM_LOWER_TILT, PinId.PWM_ARM_HAND_TURN, PinId.PWM_ARM_HAND_TILT, PinId.PWM_ARM_HAND_GRIP};

			private TwiMaster i2cBaseMaster;

			public final IRobo.Stub mBinder = new IRobo.Stub() {

				synchronized public void moveStraight(Integer leftSpeed, Integer rightSpeed) throws ConnectionLostException, InterruptedException
				{
					byte[] request = (getString(BaseProCommands.CMD_BOTH) + leftSpeed + ';' + rightSpeed).getBytes();
					byte[] response = new byte[0];
					i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				}

				synchronized public void fullStop() throws ConnectionLostException, InterruptedException
				{
					byte[] request = getString(BaseProCommands.CMD_STOP).getBytes();
					byte[] response = new byte[0];
					i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				}

				synchronized public void turnLeft(Integer leftSpeed) throws ConnectionLostException, InterruptedException
				{
					byte[] request = (getString(BaseProCommands.CMD_LEFT) + leftSpeed).getBytes();
					byte[] response = new byte[0];
					i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				}

				synchronized public void turnRight(Integer rightSpeed) throws ConnectionLostException, InterruptedException
				{
					byte[] request = (getString(BaseProCommands.CMD_RIGHT) + rightSpeed).getBytes();
					byte[] response = new byte[0];
					i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				}

				synchronized public void setServo(int servoId, int angle)
				{
					servos_.get(servoId).setDuty(angle);
				}

				synchronized public Float getCharger()
				{
					DbMsg.i("battery voltage requested");
					request = ("" + BaseProCommands.CMD_CHARGER).getBytes();
					response = new byte[50];
					mLooper.i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
					return new Float(new String(response));
				}

				synchronized public void setServo(Boolean on)
				{
					String on = Boolean.toString(on);
					request = (BaseProCommands.CMD_LED + on).getBytes();
					response = new byte[0];
					i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				}

				public String getName()
				{
					return "Track";
				}

				public String getVersion()
				{
					return "0.0";
				}

				synchronized public double getBattery()
				{
					DbMsg.i("battery voltage requested");
					request = ("" + BaseProCommands.CMD_BATTERY).getBytes();
					response = new byte[50];
					mLooper.i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
					return new Float(new String(response)).doubleValue();
				}

			};

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
				headScannerThread.start();
				i2cBaseMaster = ioio_.openTwiMaster(PinId.BASE_I2C_INDEX, TwiMaster.Rate.RATE_100KHz, false);
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
				double battery = mBinder.getBattery();
				Thread.sleep(SAMPLING_DELAY);
			}
		};


	@Override
	protected IOIOLooper createIOIOLooper() {
		return mLooper;
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
		return mLooper.mBinder;
	}

}
