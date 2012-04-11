package au.n800s.track.common;

import android.util.Log;

public class DbMsg {

	static private String TAG = "Robo";

	static public void setTag(final String tag) {
		TAG = tag;
	}

	static public void i(String msg)
	{
		Log.i(TAG, msg);
	}

	static public void i(String msg, String subgroup)
	{
		Log.i(TAG + "/" + subgroup, msg);
	}

	static public void e(String msg)
	{
		Log.e(TAG, msg);
	}

	static public void e(String msg, Exception e)
	{
		Log.e(TAG, msg, e);
	}

	static public void e(String msg, Exception e, String subgroup)
	{
		Log.e(TAG + "/" + subgroup, msg, e);
	}

}
