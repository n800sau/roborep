package au.n800s.track.rc;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;

import javax.net.ssl.SSLServerSocket;
import javax.net.ssl.SSLServerSocketFactory;
import javax.net.ssl.SSLSocket;

import android.content.Context;
import au.n800s.track.common.DbMsg;

public class TCPServerThread implements Runnable {

	private Context context;

	SSLServerSocket sslserversocket;
	boolean isRunning=false;

	ArrayList<SSLSocket> mClients = new ArrayList<SSLSocket>();

	TCPServerThread(Context context) {
		//		System.setProperty("javax.net.ssl.keyStore",keystore_path);
		//		System.setProperty("javax.net.ssl.keyStorePassword",passwd);
		this.context=context;
		isRunning=true;
	}

	public void run() {

		try {
			String host		= getLocalIpAddress();
			int port 	= 1111;

			SSLServerSocketFactory sslserversocketfactory = (SSLServerSocketFactory) SSLServerSocketFactory.getDefault();
			sslserversocket = (SSLServerSocket) sslserversocketfactory.createServerSocket(port);
			sslserversocket.setReuseAddress(true);

			while (isRunning) 
			{
				SSLSocket clientsocket = (SSLSocket) sslserversocket.accept();
				new Thread(new ClientRequestThread(this.context, clientsocket)).start();
			}
		} catch (Exception ex) {
			DbMsg.e("doInBackground Exception", ex);
		}
		DbMsg.i("TCP Server Thread finished");
	}

	public String getLocalIpAddress() {
		try {
			for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
				NetworkInterface intf = en.nextElement();
				for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
					InetAddress inetAddress = enumIpAddr.nextElement();
					if (!inetAddress.isLoopbackAddress()) {
						return inetAddress.getHostAddress().toString();
					}
				}
			}
		} catch (SocketException ex) {
			DbMsg.e("ex getLocalIpAddress", ex);
		}
		return null;
	}

	void closeConnections() {
		try {
			sslserversocket.close();
		} catch (Exception ex) {
			DbMsg.e("err closeConnections", ex);
		}

		isRunning=false;
	}

};

