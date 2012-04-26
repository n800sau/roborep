package au.n800s.robo.model.3pi;

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

import au.n800s.3pi.common.DbMsg;
import au.n800s.3pi.common.IRobo;
import au.n800s.3pi.common.IRoboCallback;

import au.n800s.3pi.common.BaseProCommands;

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

	private Uart uart;
    private InputStream in;
    private OutputStream out;

	enum MODE = {
		WALKING_AROUND
	};

	private final IRobo.Stub mBinder = new IRobo.Stub() {

		public String getName()
		{
			return "3pi";
		}

		public String getVersion()
		{
			return "0.0";
		}

		public Float getBattery()
		{
			return mLooper.getBattery()
		}

	};


	private BaseIOIOLooper mLooper = new BaseIOIOLooper() {
			Thread ledThread;
			Thread headScannerThread;

			@Override
			protected void setup() throws ConnectionLostException, InterruptedException {
				DbMsg.i("Connected.");
				uart = ioio_.openUart(UART_3PI_RX, UART_3PI_TX, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
		        in = uart.getInputStream();
		        out = uart.getOutputStream();
				ledThread = new LEDThread(ioio_);
				ledThread.start();
				DbMsg.i("Thread started");
				headScannerThread = new HeadScannerThread(ioio_);
				headScannerThread.start();
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
    
    protected String get_version() throws IOException, InterruptedException
    {
    	return new String(requestData(new byte[]{(byte)0x81}, 6), 0, 6);
    }
    
    protected short[] get_ir_raw() throws IOException, InterruptedException
    {
    	short[] rs = new short[RobotState.IR_COUNT];
    	byte[] data = requestData(new byte[]{(byte)0x86}, 10);
    	for(int i=0; i<RobotState.IR_COUNT; i++) {
    		rs[i] = (short)( ((((short)data[i*2+1])&0xFF)<<8) | (data[i*2]&0xFF) );
    	}
    	return rs;
    }

    protected short get_battery() throws IOException, InterruptedException
    {
    	byte data[] = requestData(new byte[]{(byte)0xB1}, 2);
    	return (short)( ((((short)data[1])&0xFF)<<8) | (data[0]&0xFF) );
    }

    protected void update_motors() throws IOException, InterruptedException, JSONException
    {
    	short leftspeed = (short)rstate.x_getInt("leftMotorSpeed");
    	DbMsg.i("left motor:" + leftspeed);
		byte[] left = Utils.short2bytes(leftspeed);
    	short rightspeed = (short)rstate.x_getInt("rightMotorSpeed");
    	DbMsg.i("right motor:" + rightspeed);
		byte[] right = Utils.short2bytes(rightspeed);
   		requestData(new byte[]{(byte)0xC7, left[0], left[1], right[0], right[1]}, 0);
    }
    
    protected void accelerate() throws IOException, InterruptedException, JSONException
    {
		int speed = (rstate.x_getInt("leftMotorSpeed") + rstate.x_getInt("rightMotorSpeed")) / 2;
		setSpeed(speed+10, speed+10);
    }
    
    protected void decelerate() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(-10, -10);
    }

	protected void changeSpeed(int left, int right) throws IOException, InterruptedException, JSONException 
	{
		setSpeed(rstate.x_getInt("leftMotorSpeed") + left, rstate.x_getInt("rightMotorSpeed") + right);
	}

	protected void setSpeed(int left, int right) throws IOException, InterruptedException, JSONException 
	{
		if(left > Utils.MAXVAL) {
			left = Utils.MAXVAL;
		}
		if(left < Utils.MINVAL) {
			left = Utils.MINVAL;
		}
		if(right > Utils.MAXVAL) {
			right = Utils.MAXVAL;
		}
		if(right < Utils.MINVAL) {
			right = Utils.MINVAL;
		}
    	rstate.x_put("leftMotorSpeed", left);
		rstate.x_put("rightMotorSpeed", right);
		update_motors();
	}

    protected void turn_left() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(-10, 10);
    }
    
    protected void turn_right() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(10, -10);
    }
    
    protected void stop_motor() throws IOException, InterruptedException, JSONException
    {
		setSpeed(0, 0);
    }

			synchronized public Float getBattery()
			{
				DbMsg.i("battery voltage requested");
				request = ("" + BaseProCommands.CMD_BATTERY).getBytes();
				response = new byte[50];
				mLooper.i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				return new Float(new String(response));
			}

			synchronized public Float getCharger()
			{
				DbMsg.i("battery voltage requested");
				request = ("" + BaseProCommands.CMD_CHARGER).getBytes();
				response = new byte[50];
				mLooper.i2cBaseMaster.writeRead(BaseProCommands.I2C_Addr, false, request, request.length, response, response.length);
				return new Float(new String(response));
			}

			@Override
			public void loop() throws ConnectionLostException, InterruptedException {
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
			Notification notification = new Notification(R.drawable.ic_launcher, "3pi service running", System.currentTimeMillis());
			notification.setLatestEventInfo(this, "3pi Robotarr Service", "Click to stop", PendingIntent.getService(this, 0, new Intent("stop", null, this, this.getClass()), 0));
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
