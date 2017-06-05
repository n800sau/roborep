package au.n800s.bleservice;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;

public class MainActivity extends Activity
{
	private static final String TAG = "bleservice.MainActivity";

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{

		super.onCreate(savedInstanceState);
		Log.d(TAG, "Successfully started");
		setContentView(R.layout.main);
	}
}
