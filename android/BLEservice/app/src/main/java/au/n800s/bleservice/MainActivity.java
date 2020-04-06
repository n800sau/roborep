package au.n800s.bleservice;

import android.app.Activity;
import android.os.Bundle;
import android.content.Context;
import android.util.Log;
import android.content.Intent;

public class MainActivity extends Activity
{
	private static final String TAG = "bleservice.MainActivity";

	private Context ctx = this;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{

		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		startService(new Intent(ctx, BLEService.class));
		Log.d(TAG, "Successfully started");
	}
}
