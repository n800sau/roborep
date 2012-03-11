package au.n800s.ioio.sample1;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class IOIOThread extends Thread {

	private IOIO ioio_;
	private boolean abort_ = false;
	private Uart uart;
    private InputStream in;
    private OutputStream out;
	private CommandQueue queue;
	private RobotState rstate;

    IOIOThread(CommandQueue queue, RobotState rstate) {
    	super();
    	this.queue = queue;
    	this.rstate = rstate;
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
    
    protected short get_battery() throws IOException, InterruptedException
    {
    	byte data[] = requestData(new byte[]{(byte)0xB1}, 2);
    	return (short)( ((((short)data[1])&0xFF)<<8) | (data[0]&0xFF) );
    }

    protected void left_motor() throws IOException, InterruptedException
    {
    	if (rstate.leftMotorSpeed > 0) {
    		requestData(new byte[]{(byte)0xC5, (byte)rstate.leftMotorSpeed}, 0);
    	} else {
    		requestData(new byte[]{(byte)0xC6, (byte)-rstate.leftMotorSpeed}, 0);
    	}
    }
    
    protected void right_motor() throws IOException, InterruptedException
    {
    	if (rstate.rightMotorSpeed > 0) {
    		requestData(new byte[]{(byte)0xC1, (byte)rstate.rightMotorSpeed}, 0);
    	} else {
    		requestData(new byte[]{(byte)0xC2, (byte)-rstate.rightMotorSpeed}, 0);
    	}
    }
    
    protected void accelerate() throws IOException, InterruptedException
    {
    	rstate.leftMotorSpeed = rstate.rightMotorSpeed = (rstate.leftMotorSpeed + rstate.rightMotorSpeed) / 2;  
    	rstate.leftMotorSpeed++;
    	rstate.rightMotorSpeed++;
    	left_motor();
    	right_motor();
    }
    
    protected void decelerate() throws IOException, InterruptedException
    {
    	rstate.leftMotorSpeed--;
    	rstate.rightMotorSpeed--;
    	left_motor();
    	right_motor();
    }

    protected void turn_left() throws IOException, InterruptedException
    {
    	rstate.leftMotorSpeed++;
    	rstate.rightMotorSpeed--;
    	left_motor();
    	right_motor();
    }
    
    protected void turn_right() throws IOException, InterruptedException
    {
    	rstate.leftMotorSpeed--;
    	rstate.rightMotorSpeed++;
    	left_motor();
    	right_motor();
    }
    
    protected void stop_motor() throws IOException, InterruptedException
    {
    	rstate.leftMotorSpeed = 0;
    	rstate.rightMotorSpeed = 0;
    	left_motor();
    	right_motor();
    }
    
	/** Thread body. */
	@Override
	public void run() {
		super.run();
		DbMsg.i( "run ioio");
		while (true) {
			synchronized (this) {
				if (abort_) {
					break;
				}
				ioio_ = IOIOFactory.create();
			}
			try {
				synchronized(rstate) {
					rstate.connection = false;
				}
				DbMsg.i( "waiting ioio");
				ioio_.waitForConnect();
				DbMsg.i( "connected ioio");
				synchronized(rstate) {
					rstate.connection = true;
				}
				uart = ioio_.openUart(3, 4, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
		        in = uart.getInputStream();
		        out = uart.getOutputStream();
				DigitalOutput led = ioio_.openDigitalOutput(0, true);
				//get version
				synchronized(rstate) {
					rstate.version = get_version();
				}
				synchronized(rstate) {
					DbMsg.i( "ioio version" + rstate.version);
				}
				while (true) {
					try {
						synchronized(rstate) {
							rstate.battery = get_battery();
						}
						Command command = queue.nextCommand();
						if (command != null) {
							DbMsg.i("Command:" + command.name + " len:" + command.name.length());
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
//							DbMsg.i( "led=" + String.valueOf(rstate.led));
//						}
						synchronized(rstate) {
							led.write(!rstate.led);
						}
						synchronized(rstate) {
							rstate.error = "None";
						}
					} catch (Exception e) {
						synchronized(rstate) {
							rstate.error = e.getMessage();
						}
					}
					sleep(100);
				}
			} catch (ConnectionLostException e) {
			} catch (Exception e) {
				DbMsg.e("Unexpected exception caught, ioio disconnected", e);
				ioio_.disconnect();
			} finally {
				try {
					ioio_.waitForDisconnect();
				} catch (InterruptedException e) {
				}
			}
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


}
