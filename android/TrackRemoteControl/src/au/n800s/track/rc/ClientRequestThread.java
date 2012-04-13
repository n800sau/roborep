package au.n800s.track.rc;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;

import javax.net.ssl.SSLSocket;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Messenger;
import android.os.Message;
import android.os.RemoteException;
import org.json.JSONException;
import org.json.JSONObject;

import au.n800s.track.common.DbMsg;
import au.n800s.track.common.MessageId;

public class ClientRequestThread implements Runnable {

	private Context context;
	/** Messenger for communicating with service. */
	Messenger mService = null;
	/** Flag indicating whether we have called bind on the service. */
	boolean mIsBound;

	private SSLSocket socket;
	private BufferedReader input;
	private OutputStream output;

	boolean isRunning=false;

	ClientRequestThread(Context context, SSLSocket socket) throws UnsupportedEncodingException, IOException {
		this.context=context;
		this.socket = socket;
		input = new BufferedReader(new InputStreamReader(socket.getInputStream(), "UTF-8"));
		output = socket.getOutputStream();
		isRunning=true;
	}

	public JSONObject getObjectFromInput() throws IOException, JSONException {
		JSONObject rs = null;
		String s = input.readLine();
		if( s != "") {
			rs = new JSONObject(s);
		}
		return rs;
	}

	public void sendObjectToOutput(JSONObject reply) {
		try {
			output.write(reply.toString().getBytes());
		} catch (Exception ex) {
			DbMsg.e("ex send", ex);
		}
	}

	void closeConnections() {
		try {
			input.close();
			output.close();
			socket.close();
		} catch (Exception ex) {
			DbMsg.e("err closeConnections", ex);
		}

		isRunning=false;
	}


	public void run() {
		doBindService();
		try {
			while(true) {
				JSONObject cmd = getObjectFromInput();
				if( cmd != null ) {
					DbMsg.i("received" + cmd);
					switch(cmd.getString("command")) {
						//{"command": "base_led", "on", "1"}
						//{"command": "base_led", "on", "0"}
						case "base_led":
							Message.obtain(null, MessageId.MSG_BASE_LED, cmd.getBoolean("on"));
						default:
							cmd.put("reply", true);
							sendObjectToOutput(cmd);
							break;
					}
				}
			}
		} catch(JSONException ex) {
			DbMsg.e("ClientRequestThread", ex);
		} catch (IOException ex) {
			DbMsg.e("ClientRequestThread", ex);
		} finally {
			doUnbindService();
		}
	}

	/**
	 * Handler of incoming messages from service.
	 */
	class IncomingHandler extends Handler {
		@Override
		public void handleMessage(Message msg) {
			switch (msg.what) {
			case MessageId.MSG_SET_VALUE:
				DbMsg.i("Received from service: " + msg.arg1);
				break;
			case MessageId.MSG_SERVO:
				DbMsg.i("Action fulfilled");
				break;
			default:
				super.handleMessage(msg);
			}
		}
	}

	/**
	 * Target we publish for clients to send messages to IncomingHandler.
	 */
	final Messenger mMessenger = new Messenger(new IncomingHandler());

	/**
	 * Class for interacting with the main interface of the service.
	 */
	private ServiceConnection mConnection = new ServiceConnection() {
		public void onServiceConnected(ComponentName className, IBinder service) {
			// This is called when the connection with the service has been
			// established, giving us the service object we can use to
			// interact with the service.  We are communicating with our
			// service through an IDL interface, so get a client-side
			// representation of that from the raw service object.
			mService = new Messenger(service);
			DbMsg.i("Attached.");

			// We want to monitor the service for as long as we are
			// connected to it.
			try {
				Message msg = Message.obtain(null, MessageId.MSG_REGISTER_CLIENT);
				msg.replyTo = mMessenger;
				mService.send(msg);

				// Give it some value as an example.
				msg = Message.obtain(null, MessageId.MSG_SET_VALUE, this.hashCode(), 0);
				mService.send(msg);
			} catch (RemoteException e) {
				// In this case the service has crashed before we could even
				// do anything with it; we can count on soon being
				// disconnected (and then reconnected if it can be restarted)
				// so there is no need to do anything here.
			}

			DbMsg.i(context.getString(R.string.remote_service_connected));
		}

		public void onServiceDisconnected(ComponentName className) {
			// This is called when the connection with the service has been
			// unexpectedly disconnected -- that is, its process crashed.
			mService = null;

			DbMsg.i(context.getString(R.string.remote_service_disconnected));
		}
	};

	void doBindService() {
		// Establish a connection with the service.  We use an explicit
		// class name because there is no reason to be able to let other
		// applications replace our component.
		context.bindService(new Intent("au.n800s.ioio.rserv.IOIORoboRemoteService"), mConnection, Context.BIND_AUTO_CREATE);
		mIsBound = true;
		DbMsg.i("Binding...");
	}

	void doUnbindService() {
		if (mIsBound) {
			// If we have received the service, and hence registered with
			// it, then now is the time to unregister.
			if (mService != null) {
				try {
					Message msg = Message.obtain(null, MessageId.MSG_UNREGISTER_CLIENT);
					msg.replyTo = mMessenger;
					mService.send(msg);
				} catch (RemoteException e) {
					// There is nothing special we need to do if the service
					// has crashed.
				}
			}

			// Detach our existing connection.
			context.unbindService(mConnection);
			mIsBound = false;
		}
	}

}
