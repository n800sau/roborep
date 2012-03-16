package au.n800s.ioio.sample1;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;

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

    IOIOThread(CommandQueue queue, RobotState rstate) {
    	super();
    	this.queue = queue;
    	this.rstate = rstate;
		mHandler = new Handler();
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
    	return new String(requestData(new byte[]{(byte)0x81}, 6));
    }
    
    protected short[] get_raw_sensors() throws IOException, InterruptedException
    {
    	return (short[])requestData(new byte[]{(byte)0x86}, 10);
    }

    protected short get_raw_battery() throws IOException, InterruptedException
    {
    	byte data[] = requestData(new byte[]{(byte)0xB1}, 2);
    	return (short)( ((((short)data[1])&0xFF)<<8) | (data[0]&0xFF) );
    }

    protected void left_motor() throws IOException, InterruptedException, JSONException
    {
    	if (rstate.x_getInt("rightMotorSpeed") > 0) {
    		requestData(new byte[]{(byte)0xC1, (byte)rstate.x_getInt("rightMotorSpeed")}, 0);
    	} else {
    		requestData(new byte[]{(byte)0xC2, (byte)-rstate.x_getInt("rightMotorSpeed")}, 0);
    	}
    }
    
    protected void right_motor() throws IOException, InterruptedException, JSONException
    {
    	if (rstate.x_getInt("leftMotorSpeed") > 0) {
    		requestData(new byte[]{(byte)0xC5, (byte)rstate.x_getInt("leftMotorSpeed")}, 0);
    	} else {
    		requestData(new byte[]{(byte)0xC6, (byte)-rstate.x_getInt("leftMotorSpeed")}, 0);
    	}
    }
    
    protected void accelerate() throws IOException, InterruptedException, JSONException
    {
		int speed = (rstate.x_getInt("leftMotorSpeed") + rstate.x_getInt("rightMotorSpeed")) / 2;
		setSpeed(speed+1, speed+1);
    }
    
    protected void decelerate() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(-1, -1);
    }

	protected void changeSpeed(int left, int right) throws IOException, InterruptedException, JSONException 
	{
    	rstate.x_put("leftMotorSpeed", rstate.x_getInt("leftMotorSpeed") + left);
		rstate.x_put("rightMotorSpeed", rstate.x_getInt("rightMotorSpeed") + right);
    	left_motor();
    	right_motor();
	}

	protected void setSpeed(int left, int right) throws IOException, InterruptedException, JSONException 
	{
    	rstate.x_put("leftMotorSpeed", left);
		rstate.x_put("rightMotorSpeed", right);
    	left_motor();
    	right_motor();
	}

    protected void turn_left() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(-1, 1);
    }
    
    protected void turn_right() throws IOException, InterruptedException, JSONException
    {
		changeSpeed(1, -1);
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
		mHandler.postDelayed(mPushState, 100);
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
						short[] data = get_raw_sensors();
						for(int i=0; i<5; i++) {
							rstate.x_put("raw_sensor" + i, data[i]);
						}
						if ( rstate.x_getDouble("current_heading") >= 0 ) {
							DbMsg.i( "head to " + rstate.x_getDouble("current_heading"));
							int offset = (int)((rstate.x_getDouble("current_heading") - rstate.x_getDouble("heading")) % 360);
							if(offset < 0) {
								//turn left
								changeSpeed(-offset * 2, offset * 2);
							} else if (offset > 0) {
								//turn right
								changeSpeed(offset * 2, -offset * 2);
							}
						}
						Command command = queue.nextCommand();
						if (command != null) {
							DbMsg.i("Command:" + command.name + " len:" + command.name.length());
							if (command.name.equalsIgnoreCase("set_direction")) {
								rstate.x_put("current_heading", command.params.getDouble("direction"));
							}
							if (command.name.equalsIgnoreCase("keep_direction")) {
								rstate.x_put("current_heading", rstate.getDouble("heading"));
							}
							if (command.name.equalsIgnoreCase("stop_direction")) {
								rstate.x_put("current_heading", -1);
							}
							if (command.name.equalsIgnoreCase("accelerate")) {
								accelerate();
							}
							if (command.name.equalsIgnoreCase("decelerate")) {
								decelerate();
							}
							if (command.name.equalsIgnoreCase("left")) {
								turn_left();
							}
							if (command.name.equalsIgnoreCase("right")) {
								turn_right();
							}
							if (command.name.equalsIgnoreCase("stop")) {
								DbMsg.i("stop");
								stop_motor();
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
					mHandler.postDelayed(this, 100);
				}
		   }
	};
	

}
