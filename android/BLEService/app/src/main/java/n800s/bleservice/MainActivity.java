package n800s.bleservice;

import android.Manifest;
import android.annotation.TargetApi;
import android.app.AlertDialog;
import android.app.ListActivity;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.content.Intent;
import android.content.Context;

import android.app.AlarmManager;
import android.app.PendingIntent;
import android.util.Log;

import android.os.SystemClock;
import android.support.v4.content.LocalBroadcastManager;
import android.content.BroadcastReceiver;
import android.content.IntentFilter;
import android.widget.Toast;
import android.widget.Button;
import android.view.View;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import android.os.Handler;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.widget.LinearLayout;
import android.view.LayoutInflater;
import android.widget.TextView;
import android.widget.ListView;
import android.support.v7.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

	private static final String TAG = "bleservice.MainActivity";
	private static final int intervalMillis = 1000;

	private LeDeviceListAdapter mLeDeviceListAdapter;
	private Handler mHandler;
	private BluetoothAdapter mBluetoothAdapter;
	private LinearLayout panel;
	private ListView lv;

	final private int REQUEST_CODE_ASK_MULTIPLE_PERMISSIONS = 124;
	private static final int REQUEST_ENABLE_BT = 1;

	private Context ctx = this;

	private Intent serviceIntent;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		panel = (LinearLayout)findViewById(R.id.outer_layout);
		lv = (ListView)findViewById(R.id.listview);

		// Use this check to determine whether BLE is supported on the device.  Then you can
		// selectively disable BLE-related features.
		if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
			Toast.makeText(this, R.string.ble_not_supported, Toast.LENGTH_SHORT).show();
			finish();
		}

		if (Build.VERSION.SDK_INT >= 23) {
			// Marshmallow+ Permission APIs
			fuckMarshMallow();
		}

		// Initializes a Bluetooth adapter.  For API level 18 and above, get a reference to
		// BluetoothAdapter through BluetoothManager.
		final BluetoothManager bluetoothManager =
				(BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
		mBluetoothAdapter = bluetoothManager.getAdapter();
		// Ensures Bluetooth is enabled on the device.  If Bluetooth is not currently enabled,
		// fire an intent to display a dialog asking the user to grant permission to enable it.
		if (!mBluetoothAdapter.isEnabled()) {
			if (!mBluetoothAdapter.isEnabled()) {
				Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
				startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
			}
		}

		// Initializes list view adapter.
		mLeDeviceListAdapter = new LeDeviceListAdapter();
		lv.setAdapter(mLeDeviceListAdapter);

	serviceIntent = new Intent(this, BLEService.class);

	Button btn = (Button)findViewById(R.id.button);
	btn.setOnClickListener(new View.OnClickListener() {
	  @Override public void onClick(View view) {
		mLeDeviceListAdapter.clear();
		startService(serviceIntent);
	  }
	});

	LocalBroadcastManager.getInstance(this).registerReceiver(messageBroadcastReceiver, new IntentFilter("ble_message"));
  }

	protected void onDestroy() {
		super.onDestroy();
		LocalBroadcastManager.getInstance(this).unregisterReceiver(messageBroadcastReceiver);
	}

	@Override
	public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
		switch (requestCode) {
			case REQUEST_CODE_ASK_MULTIPLE_PERMISSIONS: {
				Map<String, Integer> perms = new HashMap<String, Integer>();
				// Initial
				perms.put(Manifest.permission.ACCESS_FINE_LOCATION, PackageManager.PERMISSION_GRANTED);


				// Fill with results
				for (int i = 0; i < permissions.length; i++)
					perms.put(permissions[i], grantResults[i]);

				// Check for ACCESS_FINE_LOCATION
				if (perms.get(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED

						) {
					// All Permissions Granted

					// Permission Denied
					Toast.makeText(this, "All Permission GRANTED !! Thank You :)", Toast.LENGTH_SHORT)
							.show();


				} else {
					// Permission Denied
					Toast.makeText(this, "One or More Permissions are DENIED Exiting App :(", Toast.LENGTH_SHORT)
							.show();

					finish();
				}
			}
			break;
			default:
				super.onRequestPermissionsResult(requestCode, permissions, grantResults);
		}
	}

	@Override
	protected void onResume() {
		super.onResume();
	}

	@Override
	protected void onPause() {
		super.onPause();
	}


	@TargetApi(Build.VERSION_CODES.M)
	private void fuckMarshMallow() {
		List<String> permissionsNeeded = new ArrayList<String>();

		final List<String> permissionsList = new ArrayList<String>();
		if (!addPermission(permissionsList, Manifest.permission.ACCESS_FINE_LOCATION))
			permissionsNeeded.add("Show Location");

		if (permissionsList.size() > 0) {
			if (permissionsNeeded.size() > 0) {

				// Need Rationale
				String message = "App need access to " + permissionsNeeded.get(0);

				for (int i = 1; i < permissionsNeeded.size(); i++)
					message = message + ", " + permissionsNeeded.get(i);

				showMessageOKCancel(message,
						new DialogInterface.OnClickListener() {

							@Override
							public void onClick(DialogInterface dialog, int which) {
								requestPermissions(permissionsList.toArray(new String[permissionsList.size()]),
										REQUEST_CODE_ASK_MULTIPLE_PERMISSIONS);
							}
						});
				return;
			}
			requestPermissions(permissionsList.toArray(new String[permissionsList.size()]),
					REQUEST_CODE_ASK_MULTIPLE_PERMISSIONS);
			return;
		}

		Toast.makeText(this, "No new Permission Required- Launching App .You are Awesome!!", Toast.LENGTH_SHORT)
				.show();
	}

	private void showMessageOKCancel(String message, DialogInterface.OnClickListener okListener) {
		new AlertDialog.Builder(this)
				.setMessage(message)
				.setPositiveButton("OK", okListener)
				.setNegativeButton("Cancel", null)
				.create()
				.show();
	}

	@TargetApi(Build.VERSION_CODES.M)
	private boolean addPermission(List<String> permissionsList, String permission) {

		if (checkSelfPermission(permission) != PackageManager.PERMISSION_GRANTED) {
			permissionsList.add(permission);
			// Check for Rationale Option
			if (!shouldShowRequestPermissionRationale(permission))
				return false;
		}
		return true;
	}

	// Our handler for received Intents. This will be called whenever an Intent
	// with an action named "custom-event-name" is broadcasted.
	private BroadcastReceiver messageBroadcastReceiver = new BroadcastReceiver() {
		@Override public void onReceive(Context context, Intent intent) {
			// Get extra data included in the Intent
			String message = intent.getStringExtra("message");
			DeviceDataHolder dh = (DeviceDataHolder)intent.getSerializableExtra("device");
			Toast.makeText(MainActivity.this, "Got message: " + message, Toast.LENGTH_SHORT).show();
			Log.d("receiver", "Got message: " + message);
			if(dh != null) {
				Toast.makeText(MainActivity.this, "Got device: " + dh.deviceName, Toast.LENGTH_SHORT).show();
			}
		}
	};

	class DeviceHolder {
		BluetoothDevice device;
		String additionalData;
		int rssi;
		
		public DeviceHolder(BluetoothDevice device, String additionalData, int rssi) {
			this.device = device;
			this.additionalData = additionalData;
			this.rssi = rssi;
		}
	}

	static class DeviceViewHolder {
		TextView deviceName;
		TextView deviceAd;
		TextView deviceRssi;
		TextView deviceAddress;
	}

	// Adapter for holding devices found through scanning.
	private class LeDeviceListAdapter extends BaseAdapter {
		private ArrayList<BluetoothDevice> mLeDevices;
		private ArrayList<DeviceHolder> mLeHolders;
		private LayoutInflater mInflator;

		public LeDeviceListAdapter() {
			super();
			mLeDevices = new ArrayList<BluetoothDevice>();
			mLeHolders = new ArrayList<DeviceHolder>();
			mInflator = MainActivity.this.getLayoutInflater();
		}

		public void addDevice(DeviceHolder deviceHolder) {
			if(!mLeDevices.contains(deviceHolder.device)) {
				mLeDevices.add(deviceHolder.device);
				mLeHolders.add(deviceHolder);
			}
		}

		public BluetoothDevice getDevice(int position) {
			return mLeDevices.get(position);
		}

		public void clear() {
			mLeDevices.clear();
			mLeHolders.clear();
		}

		@Override
		public int getCount() {
			return mLeDevices.size();
		}

		@Override
		public Object getItem(int i) {
			return mLeDevices.get(i);
		}

		@Override
		public long getItemId(int i) {
			return i;
		}

		@Override
		public View getView(int i, View view, ViewGroup viewGroup) {
			DeviceViewHolder viewHolder;

			BluetoothDevice device = mLeDevices.get(i);
			DeviceHolder deviceHolder = mLeHolders.get(i);
			final String deviceName = device.getName();
			if (deviceName != null && deviceName.length() > 0)
				viewHolder.deviceName.setText(deviceName);
			else
				viewHolder.deviceName.setText(R.string.unknown_device);
			viewHolder.deviceAddress.setText(device.getAddress());
			viewHolder.deviceAd.setText(deviceHolder.additionalData);
			viewHolder.deviceRssi.setText("rssi: "+Integer.toString(deviceHolder.rssi));
			return view;
		}
	}


}
