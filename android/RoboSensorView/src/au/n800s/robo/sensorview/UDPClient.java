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
	protected Integer local_port;
	public boolean start;



	public UDPClient(Handler handler, Integer local_port, String ip, Integer port)
	{
		this.Handler = handler;
		this.ip = ip;
		this.port = port;
		this.local_port = local_port;
		this.start = false;
	}

	@Override
	public void run() {
		while(true)
		{
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
				DatagramSocket socket = new DatagramSocket(local_port);
				byte[] buf;
				buf = ("{\"cmd\": \"send_full_data\", \"interval\": 100, \"count\": 20}").getBytes();
				DatagramPacket packet = new DatagramPacket(buf, buf.length, serverAddr, port);
				updatetrack("Client: Sending '" + new String(buf) + "'\n");
				socket.send(packet);
				updatetrack("Client: Message sent\n");

				byte[] messageBytes = new byte[1500];
				DatagramPacket p = new DatagramPacket(messageBytes, messageBytes.length);
				updatetrack("Client: Receiving ...\n");
				socket.receive(p);
				Integer msg_length = p.getLength();
				String message = new String(messageBytes, 0, msg_length);
				updatetrack(message + " (" + msg_length + ")");
			} catch (Exception e) {
				updatetrack("Client: Error!\n");
			}
			start = false;
		}
	}

	public void updatetrack(String s){
		Message msg = new Message();
		String textTochange = s;
		msg.obj = textTochange;
		Handler.sendMessage(msg);
	}

}
