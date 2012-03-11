package au.n800s.ioio.sample1;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPThread extends Thread {

	private boolean abort_ = false;
	private CommandQueue queue;
	private RobotState rstate;
	
    UDPThread(CommandQueue queue, RobotState rstate) {
    	super();
    	this.queue = queue;
    	this.rstate = rstate;
    }

    /** Thread body. */
	@Override
	public void run() {
		super.run();
		while (true) {
			try {
            	DbMsg.i("UDP socket opened", "UDP");
				DatagramSocket serverSocket = new DatagramSocket(9876);
				try {
		            byte[] receiveData = new byte[1024];
		            byte[] sendData = new byte[1024];
		            while (true) {
						synchronized (this) {
							if (abort_) {
								break;
							}
						}
		            	DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
		            	serverSocket.receive(receivePacket);
		            	String sentence = new String( receivePacket.getData()).substring(0, receivePacket.getLength());
		            	DbMsg.i("Write command", "UDP");
		            	String reply="";
		            	Command cmd = new Command(sentence.toLowerCase());
		            	if (cmd.name.equalsIgnoreCase("version")) {
		            		reply = rstate.version;
		            	} else if (cmd.name.equalsIgnoreCase("battery")) {
			            	reply = String.valueOf(rstate.battery);
		            	} else if (cmd.name.equalsIgnoreCase("orientation")) {
			            	reply = String.valueOf(rstate.pitch) + ":" + String.valueOf(rstate.roll) + ":" + String.valueOf(rstate.heading);
		            	} else {
		            		queue.addCommand(cmd);
		            		DbMsg.i("RECEIVED:: " + sentence, "UDP");
		            	}
		            	InetAddress IPAddress = receivePacket.getAddress();
		            	DbMsg.i("from " + IPAddress.toString(), "UDP");
		            	int port = receivePacket.getPort();
		            	DbMsg.i("port " + port, "UDP");
		            	sendData = reply.getBytes();
		            	DatagramPacket sendPacket =	new DatagramPacket(sendData, sendData.length, IPAddress, port);
		                serverSocket.send(sendPacket);
		            }
				
				} finally {
					serverSocket.close();
				}
			} catch (Exception e) {
				DbMsg.e("Unexpected exception caught", e, "UDP");
			}
		}
	}
	synchronized public void abort() {
		abort_ = true;
	}

}
