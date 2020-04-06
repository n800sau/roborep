package n800s.bleservice;

import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
//import android.widget.Toast;
import android.util.Log;
import android.os.Handler;
import java.lang.Runnable;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.support.v4.content.LocalBroadcastManager;
import android.app.Notification;
import android.app.NotificationManager;
import java.io.Serializable;

import hu.uw.pallergabor.ble.adparser.AdElement;
import hu.uw.pallergabor.ble.adparser.AdParser;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * The Class BLEService.
 */
public class BLEService extends Service {

	private static final String TAG = "bleservice.BLEService";
    private BluetoothAdapter mBluetoothAdapter;
    private Handler mHandler;
    private boolean mScanning;

	private ArrayList <DeviceDataHolder> devlist;
	private HashMap devmap;

	private int NOTIFICATION_ID = 1;

	/** The ctx. */
	public Context ctx = this;

    // Stops scanning after 100 seconds.
    private static final long SCAN_PERIOD = 100000;

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
		mHandler = new Handler();
		devlist = new ArrayList<DeviceDataHolder>();
		devmap = new HashMap();
		// Display a notification about us starting. We put an icon in the
		// status bar.
		showNotification();
		sendMessage("The Service created");
//		Toast.makeText(this, "The Service created", Toast.LENGTH_SHORT).show();

        // Initializes a Bluetooth adapter.  For API level 18 and above, get a reference to
        // BluetoothAdapter through BluetoothManager.
        final BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(this.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();

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

       scanLeDevice(true);
		return START_STICKY;
	}

	/**
	 * Ran when when the service is destroyed
	 */
	@Override
	public void onDestroy() {

        NotificationManager notificationManager =
                (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        notificationManager.cancel(NOTIFICATION_ID);
		// Tell the user we stopped.
//		Toast.makeText(this, "The Service has Stopped", Toast.LENGTH_SHORT).show();
		sendMessage("The Service has Stopped");
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
//		Toast.makeText(this, text.toString(), Toast.LENGTH_SHORT).show();
		// This is the intent that is fired off after the user clicks the
		// notification
		PendingIntent contentIntent = PendingIntent.getActivity(this, 0, new Intent(this, MainActivity.class), 0);

Notification.Builder builder = new Notification.Builder(this)
            .setContentIntent(contentIntent)
            .setSmallIcon(R.mipmap.ic_launcher)
            .setContentTitle(text);
    Notification notification = builder.build();


//		Notification notification = new Notification(R.mipmap.ic_launcher, text, System.currentTimeMillis());
		// This is the intent that is fired off after the user clicks the
		// notification
//		notification.setLatestEventInfo(this, text, "Click to stop the service...", contentIntent);
        NotificationManager nNM = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);

		nNM.notify(NOTIFICATION_ID, notification);



		sendMessage(text.toString());

	}

	private void scanLeDevice(final boolean enable) {
		if (enable) {
			// Stops scanning after a pre-defined scan period.
			mHandler.postDelayed(new Runnable() {
				@Override
				public void run() {
					mScanning = false;
			sendMessage("scanLeDevice stop on timeout");
					mBluetoothAdapter.stopLeScan(mLeScanCallback);
				}
			}, SCAN_PERIOD);

			mScanning = true;
			sendMessage("scanLeDevice start");
			mBluetoothAdapter.startLeScan(mLeScanCallback);
		} else {
			mScanning = false;
			sendMessage("scanLeDevice stop");
			mBluetoothAdapter.stopLeScan(mLeScanCallback);
		}
	}

	private BluetoothAdapter.LeScanCallback mLeScanCallback =
			new BluetoothAdapter.LeScanCallback() {

		@Override
		public void onLeScan(final BluetoothDevice device, int rssi, byte[] scanRecord) {
			String deviceName = device.getName();
			sendMessage("Found " + deviceName, device, rssi, scanRecord);
		}
	};

  // Send an Intent with an action named "custom-event-name". The Intent
  // sent should
  // be received by the ReceiverActivity.
  private void sendMessage(String message) {
    Log.d(TAG, message);
    Intent intent = new Intent("ble_message");
    // You can also include some extra data.
    intent.putExtra("message", message);
    LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
  }

  private void sendMessage(String message, BluetoothDevice device, int rssi, byte[] scanRecord) {
    Log.d(TAG, message);
	DeviceDataHolder dataHolder = new DeviceDataHolder();
	dataHolder.deviceName = device.getName();
	dataHolder.deviceAddress = device.getAddress();
	ArrayList<AdElement> ads = AdParser.parseAdData(scanRecord);
	StringBuffer sb = new StringBuffer();
	for( int i = 0 ; i < ads.size() ; ++i ) {
		AdElement e = ads.get(i);
		if( i > 0 )
			sb.append(" ; ");
		sb.append(e.toString());
	}
	dataHolder.deviceAd = new String( sb );
	dataHolder.deviceRssi = "rssi: "+Integer.toString(rssi);
	devlist.add(dataHolder);
	if(devmap.get(dataHolder.deviceName) == null) {
		devmap.put(dataHolder.deviceName, dataHolder);
	    Intent intent = new Intent("ble_message");
	    // You can also include some extra data.
	    intent.putExtra("message", message);
		intent.putExtra("device", (Serializable)dataHolder);
		LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
	}
  }

}
