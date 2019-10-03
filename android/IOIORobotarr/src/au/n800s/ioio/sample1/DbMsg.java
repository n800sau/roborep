package au.n800s.ioio.sample1;

import android.util.Log;

public class DbMsg {

	static String TAG = "Robotarr";
	
	static void i(String msg)
	{
		Log.i(TAG, msg);
	}
	
	static void i(String msg, String subgroup)
	{
		Log.i(TAG + "/" + subgroup, msg);
	}
	
	static void e(String msg)
	{
		Log.e(TAG, msg);
	}
	
	static void e(String msg, Exception e)
	{
		Log.e(TAG, msg, e);
	}
	
	static void e(String msg, Exception e, String subgroup)
	{
		Log.e(TAG + "/" + subgroup, msg, e);
	}
	
}
