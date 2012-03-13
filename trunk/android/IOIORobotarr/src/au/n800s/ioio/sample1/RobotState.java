package au.n800s.ioio.sample1;

import org.json.JSONObject;
import org.json.JSONException;

public class RobotState extends JSONObject {
	

	RobotState() throws JSONException 
	{
		put("connection", false);
		put("version", "");
		put("leftMotorSpeed", 0);
		put("rightMotorSpeed", 0);
		put("battery", 0);
		put("led", false);
		put("pitch", 25.);
		put("roll", 60.);
		put("heading", 90.);
		put("current_heading", -1);
		put("error", "");
	}

	synchronized public void x_put(String key, String value) throws JSONException 
	{
		put(key, value);
	}

	synchronized public void x_put(String key, boolean value) throws JSONException 
	{
		put(key, value);
	}

	synchronized public void x_put(String key, double value) throws JSONException 
	{
		put(key, value);
	}

	synchronized public void x_put(String key, int value) throws JSONException 
	{
		put(key, value);
	}

	synchronized public String x_getString(String key) throws JSONException 
	{
		return getString(key);
	}
	
	synchronized public boolean x_getBoolean(String key) throws JSONException 
	{
		return getBoolean(key);
	}
	
	synchronized public int x_getInt(String key) throws JSONException 
	{
		return getInt(key);
	}
	
	synchronized public double x_getDouble(String key) throws JSONException
	{
		return getDouble(key);
	}
	
};
