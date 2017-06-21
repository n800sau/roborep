package n800s.bleservice;

import android.support.v7.app.AppCompatActivity;
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

public class MainActivity extends AppCompatActivity {

	private static final String TAG = "bleservice.MainActivity";
	private static final int intervalMillis = 1000;

	private Context ctx = this;

	private Intent serviceIntent;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		serviceIntent = new Intent(this, BLEService.class);

		Button btn = (Button) findViewById(R.id.button);
		btn.setOnClickListener(new View.OnClickListener() {
			@Override public void onClick(View view) {
				startService(serviceIntent);
			}
		});

		// Register to receive messages.
		// We are registering an observer (mMessageReceiver) to receive Intents
		// with actions named "custom-event-name".
		LocalBroadcastManager.getInstance(this).registerReceiver(messageBroadcastReceiver, new IntentFilter("ble_message"));

	}


	protected void onDestroy() {
		super.onDestroy();
		LocalBroadcastManager.getInstance(this).unregisterReceiver(messageBroadcastReceiver);
	}


	// Our handler for received Intents. This will be called whenever an Intent
	// with an action named "custom-event-name" is broadcasted.
	private BroadcastReceiver messageBroadcastReceiver = new BroadcastReceiver() {
		@Override public void onReceive(Context context, Intent intent) {
			// Get extra data included in the Intent
			String message = intent.getStringExtra("message");
			Toast.makeText(MainActivity.this, "Got message: " + message, Toast.LENGTH_SHORT).show();
			Log.d("receiver", "Got message: " + message);
		}
	};



}
