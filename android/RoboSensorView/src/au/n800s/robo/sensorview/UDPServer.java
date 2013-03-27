package au.n800s.robo.sensorview;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import android.os.Message;
import android.os.Handler;

public class UDPServer implements Runnable {

	protected Handler Handler;
	protected String ip;
	protected Integer port;
	public boolean start;

	public UDPServer(Handler handler, String ip, Integer port)
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
			InetAddress serverAddr = InetAddress.getByName(ip);
			updatetrack("\nServer: Start connecting\n");
			DatagramSocket socket = new DatagramSocket(port, serverAddr);
			byte[] buf = new byte[17];
			DatagramPacket packet = new DatagramPacket(buf, buf.length);
			updatetrack("Server: Receiving\n");
			socket.receive(packet);
			updatetrack("Server: Message received: '" + new String(packet.getData()) + "'\n");
			updatetrack("Server: Succeed!\n");
		} catch (Exception e) {
			updatetrack("Server: Error!\n");
		}
	}

	public void updatetrack(String s){
		Message msg = new Message();
		String textTochange = s;
		msg.obj = textTochange;
		Handler.sendMessage(msg);
	}

}
