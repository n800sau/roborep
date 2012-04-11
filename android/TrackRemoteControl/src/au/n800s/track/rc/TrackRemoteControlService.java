package au.n800s.track.rc;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.os.Messenger;

import android.widget.Toast;

import au.n800s.track.common.DbMsg;

public class TrackRemoteControlService extends Service {

	/** For showing and hiding our notification. */
	NotificationManager mNM;

	/** Messenger for communicating with service. */
	Messenger mService = null;

	private TCPServerThread serverThread;

	@Override
	public void onCreate() {
		DbMsg.setTag("RoboRC");
		super.onCreate();
		mNM = (NotificationManager)getSystemService(NOTIFICATION_SERVICE);
		new Thread(serverThread = new TCPServerThread(this)).start();
		// Tell the user we started.
		Toast.makeText(this, R.string.remote_service_started, Toast.LENGTH_SHORT).show();
	}

	@Override
	public void onDestroy() {
		// Cancel the persistent notification.
		mNM.cancel(R.string.remote_service_started);

		try {
			serverThread.closeConnections();
		} catch (Exception ex) {
			DbMsg.e("ex", ex);
		}

		// Tell the user we stopped.
		Toast.makeText(this, R.string.remote_service_stopped, Toast.LENGTH_SHORT).show();

		super.onDestroy();
	}

	@Override
	public void onStart(Intent intent, int startId) {
		super.onStart(intent, startId);
		if (intent != null && intent.getAction() != null && intent.getAction().equals("stop")) {
			// User clicked the notification. Need to stop the service.
			mNM.cancel(0);
			stopSelf();
		} else {
			// Service starting. Create a notification.
			Notification notification = new Notification(R.drawable.ic_launcher, "Robotarr RC service running", System.currentTimeMillis());
			notification.setLatestEventInfo(this, "Robotarr RC Service", "Click to stop", PendingIntent.getService(this, 0, new Intent("stop", null, this, this.getClass()), 0));
			notification.flags |= Notification.FLAG_ONGOING_EVENT;
			mNM.notify(0, notification);
		}
	}

	@Override
	public IBinder onBind(Intent arg0) {
		return null;
	}


}
