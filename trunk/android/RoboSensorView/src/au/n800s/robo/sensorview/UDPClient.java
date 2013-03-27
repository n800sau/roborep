package au.n800s.robo.sensorview;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import android.os.Message;
import android.os.Handler;

public class UDPClient implements Runnable {

	protected Handler Handler;
	protected String ip;
	protected Integer port;
	public boolean start;

	public UDPClient(Handler handler, String ip, Integer port)
	{
		this.Handler = handler;
		this.ip = ip;
		this.port = port;
		this.start = false;
	}

	@Override
	public void run() {
		while(start==false)
		{
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		try {
			InetAddress serverAddr = InetAddress.getByName(ip);
			updatetrack("Client: Start connecting\n");
			DatagramSocket socket = new DatagramSocket();
			byte[] buf;
			buf = ("Default message").getBytes();
			DatagramPacket packet = new DatagramPacket(buf, buf.length, serverAddr, port);
			updatetrack("Client: Sending '" + new String(buf) + "'\n");
			socket.send(packet);
			updatetrack("Client: Message sent\n");
			updatetrack("Client: Succeed!\n");

			byte[] messageBytes = new byte[1500];
			DatagramPacket p = new DatagramPacket(messageBytes, messageBytes.length);
			socket.receive(p);
			Integer msg_length = p.getLength();  
			String message = new String(messageBytes, 0, msg_length);  
			updatetrack(message + " (" + msg_length + ")");
		} catch (Exception e) {
			updatetrack("Client: Error!\n");
		}
	}

	public void updatetrack(String s){
		Message msg = new Message();
		String textTochange = s;
		msg.obj = textTochange;
		Handler.sendMessage(msg);
	}

}
