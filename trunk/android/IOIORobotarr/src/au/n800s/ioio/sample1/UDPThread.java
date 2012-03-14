package au.n800s.ioio.sample1;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.json.JSONObject;

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
            	DbMsg.i("UDP socket opening", "UDP");
				DatagramSocket serverSocket = new DatagramSocket(9876);
            	DbMsg.i("UDP socket opened", "UDP");
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
						JSONObject jsonObject = new JSONObject(sentence);
		            	DbMsg.i("Write command", "UDP");
						JSONObject reply = new JSONObject();
		            	Command cmd = new Command(jsonObject.getString("command"), jsonObject);
		            	if (cmd.name.equalsIgnoreCase("state")) {
		            		reply = rstate.state();
						} else {
		            		queue.addCommand(cmd);
		            		DbMsg.i("RECEIVED:: " + jsonObject.toString(), "UDP");
		            	}
		            	InetAddress IPAddress = receivePacket.getAddress();
		            	DbMsg.i("from " + IPAddress.toString(), "UDP");
		            	int port = receivePacket.getPort();
		            	DbMsg.i("port " + port, "UDP");
		            	sendData = reply.toString().getBytes();
		            	DatagramPacket sendPacket =	new DatagramPacket(sendData, sendData.length, IPAddress, port);
		                serverSocket.send(sendPacket);
		            }
				
				} finally {
	            	DbMsg.i("UDP socket closing", "UDP");
					serverSocket.close();
	            	DbMsg.i("UDP socket closed", "UDP");
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
