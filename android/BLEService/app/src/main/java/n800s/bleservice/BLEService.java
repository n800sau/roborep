package n800s.bleservice;

import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.widget.Toast;
import android.util.Log;
import android.os.Handler;
import java.lang.Runnable;

/**
 * The Class BLEService.
 */
public class BLEService extends Service {

	private static final String TAG = "bleservice.BLEService";

	/** The ctx. */
	public Context ctx = this;

	/**
	 * Class for clients to access. Because we know this service always runs in
	 * the same process as its clients, we don't need to deal with Inter-Process
	 * Communication.
	 */
	public class LocalBinder extends Binder {

		/**
		 * Gets the service.
		 * 
		 * @return the service
		 */
		BLEService getService() {
			return BLEService.this;
		}
	}

	/**
	 * Creates the service
	 */
	@Override
	public void onCreate() {
		// Display a notification about us starting. We put an icon in the
		// status bar.
		showNotification();
		Log.d(TAG, "The Service created");
		new Handler().postDelayed(new Runnable() {
			@Override
			public void run() {
				stopSelf();
			}
		}, 10000);
	}

	/**
	 * onStartCommand is called when service is first created. This is where we
	 * put the code that does the work for the service
	 */
	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		// We want this service to continue running until it is explicitly
		// stopped, so return sticky.

		// This is the method we would put the code that does the work

		return START_STICKY;
	}

	/**
	 * Ran when when the service is destroyed
	 */
	@Override
	public void onDestroy() {

		// Tell the user we stopped.
		Toast.makeText(this, "The Service has Stopped", Toast.LENGTH_SHORT).show();
		Log.d(TAG, "The Service has Stopped");
	}

	/**
	 * Return the local binder
	 */
	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}

	// This is the object that receives interactions from clients. See
	// RemoteService for a more complete example.
	/** The m binder. */
	private final IBinder mBinder = new LocalBinder();

	/**
	 * Show a notification while this service is running.
	 */
	private void showNotification() {
		CharSequence text = "Service has started...";
		// This is the intent that is fired off after the user clicks the
		// notification
		PendingIntent contentIntent = PendingIntent.getActivity(this, 0,
				new Intent(this, StopServiceActivity.class), 0);
		Log.d(TAG, text.toString());

	}

}
