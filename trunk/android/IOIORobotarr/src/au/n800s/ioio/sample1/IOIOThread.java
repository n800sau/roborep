package au.n800s.ioio.sample1;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.api.PwmOutput;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import org.json.JSONException;

import android.os.Handler;

public class IOIOThread extends Thread {

	private IOIO ioio_;
	private boolean abort_ = false;
	private Uart uart;
    private InputStream in;
    private OutputStream out;
	private CommandQueue queue;
	private RobotState rstate;
	private Handler mHandler;
	private PwmOutput[] pwms  = new PwmOutput[RobotState.PWM_COUNT];

    IOIOThread(CommandQueue queue, RobotState rstate) throws ConnectionLostException {
    	super();
    	this.queue = queue;
    	this.rstate = rstate;
		mHandler = new Handler();
		for(int i=0; i< 5; i++) {
			pwms[i] = ioio_.openPwmOutput(12 + i, 100);
		}
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

	/** Thread body. */
	@Override
	public void run() {
		super.run();
		DbMsg.i( "run ioio");
		mHandler.postDelayed(mPushState, 1000);
		//create it here to be able to disconnect later
		ioio_ = IOIOFactory.create();
		while (true) {
			synchronized (this) {
				if (abort_) {
					break;
				}
			}
			try {
				//create it here again to be able to recriate after exceptions
				ioio_ = IOIOFactory.create();
				rstate.x_put("connection", false);
				DbMsg.i( "waiting ioio");
				ioio_.waitForConnect();
				DbMsg.i( "connected ioio");
				rstate.x_put("connection", true);
				uart = ioio_.openUart(3, 4, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
		        in = uart.getInputStream();
		        out = uart.getOutputStream();
				DigitalOutput led = ioio_.openDigitalOutput(0, true);
				//get version
				rstate.x_put("version", get_version());
				while (true) {
					try {
						rstate.x_put("battery", get_battery());
						short[] data = get_ir_raw();
						for(int i=0; i<RobotState.IR_COUNT; i++) {
							rstate.x_put("ir_raw" + i, data[i]);
						}
//						for(int i=0; i<RobotState.PWM_COUNT; i++) {
//							pwm[i].setPulseWidth(rstate.x_getInt("pwm" + i + "_pulse"));
//						}
						if ( rstate.x_getDouble("current_heading") >= 0 ) {
							DbMsg.i( "head to " + rstate.x_getDouble("current_heading"));
							int offset = (int)((rstate.x_getDouble("current_heading") - rstate.x_getDouble("heading")) % 360);
							if(offset < 0) {
								//turn left
								changeSpeed(-5, 5);
							} else if (offset > 0) {
								//turn right
								changeSpeed(5, -5);
							}
						}
						Command command = queue.nextCommand();
						if (command != null) {
							DbMsg.i("Command:" + command.name + " len:" + command.name.length());
							if (command.name.equalsIgnoreCase("set_direction")) {
								rstate.x_put("current_heading", command.params.getDouble("direction"));
							} else if (command.name.equalsIgnoreCase("keep_direction")) {
								rstate.x_put("current_heading", rstate.getDouble("heading"));
							} else if (command.name.equalsIgnoreCase("stop_direction")) {
								rstate.x_put("current_heading", -1);
							} else if (command.name.equalsIgnoreCase("accelerate")) {
								accelerate();
							} else if (command.name.equalsIgnoreCase("decelerate")) {
								decelerate();
							} else if (command.name.equalsIgnoreCase("left")) {
								turn_left();
							} else if (command.name.equalsIgnoreCase("right")) {
								turn_right();
							} else if (command.name.equalsIgnoreCase("stop")) {
								DbMsg.i("stop");
								stop_motor();
							} else if (command.name.equalsIgnoreCase("set_pwm_pulse")) {
								DbMsg.i("set pwm " + command.params.get("pwmid") + " to  pulse " + command.params.get("pulse"));
								rstate.x_put("pwm" + command.params.get("pwmid") + "_pulse", command.params.get("pulse"));
							}
						}
						//DbMsg.i( "ioio led set");
//						synchronized(rstate) {
//							DbMsg.i( "led=" + rstate.getString("led");
//						}
						led.write(!rstate.x_getBoolean("led"));
						rstate.x_put("error", "None");
					} catch (Exception e) {
						rstate.x_put("error", e.getMessage());
					}
					sleep(10);
				}
			} catch (ConnectionLostException e) {
			} catch (Exception e) {
				DbMsg.e("Unexpected exception caught, ioio disconnected", e);
				ioio_.disconnect();
			}
		}
		//disconnection
		try {
			ioio_.waitForDisconnect();
		} catch (InterruptedException e) {
		}
	}

	/**
	 * Abort the connection.
	 * 
	 * This is a little tricky synchronization-wise: we need to be handle
	 * the case of abortion happening before the IOIO instance is created or
	 * during its creation.
	 */
	synchronized public void abort() {
		abort_ = true;
		if (ioio_ != null) {
			ioio_.disconnect();
		}
	}

	private Runnable mPushState = new Runnable() 
	{
		   public void run()
		   {
				try {
					rstate.x_pushState();
				} catch(JSONException e) {
					DbMsg.e("mPushState",e);
				} finally {
					mHandler.postDelayed(this, 1000);
				}
		   }
	};
	

}
