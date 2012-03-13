package au.n800s.ioio.sample1;

import org.json.JSONObject;

public class RobotState(JSONObject) {
	

	RobotState() {
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

	synchronized public void x_put(String key, String value) {
		put(key, value);
	}

	synchronized public void x_put(String key, boolean value) {
		put(key, value);
	}

	synchronized public void x_put(String key, double value) {
		put(key, value);
	}

	synchronized public void x_put(String key, int value) {
		put(key, value);
	}

	synchronized public String x_getString(String key) {
		return getString(key);
	}
	
	synchronized public boolean x_getBoolean(String key) {
		return getBoolean(key);
	}
	
	synchronized public int x_getInt(String key) {
		return getInt(key);
	}
	
	synchronized public double x_getDouble(String key) {
		return getDouble(key);
	}
	
}
