package au.n800s.ioio.sample1;

import java.util.Calendar;
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;

public class RobotState extends JSONObject {
	
	private JSONArray history;
	private JSONObject st;
	int history_offset;

	RobotState()
	{
		history_offset = 0;
		history = new JSONArray();
		st = new JSONObject();
		st.put("connection", false);
		st.put("version", "");
		st.put("leftMotorSpeed", 0);
		st.put("rightMotorSpeed", 0);
		st.put("battery", 0);
		st.put("led", false);
		st.put("pitch", 0.);
		st.put("roll", 0.);
		st.put("heading", 0.);
		st.put("Gx", 0);
		st.put("Gy", 0);
		st.put("Gz", 0);
		st.put("current_heading", -1);
		st.put("error", "");
		st.put("index", history.length());
		st.put("timestamp", Calendar.getInstance().time)
	}


	synchronized protected void x_pushState()
	{
		st.put("timestamp", Calendar.getInstance().time)
		while(history.length() > 10000) {
			history.remove(0);
			history_offset++;
		}
		history.put(st);
		st = new JSONObject(st, st.keys());
		st.put("index", history.length());
	}

	synchronized public JSONObject x_current_state()
	{
		return new JSONObject(st, st.keys());
	}

	synchronized public JSONArray x_history(int startIndex)
	{
		JSONArray rs = new JSONArray();
		int index = (startIndex - history_offset;
		if(index < 0) {
			index = 0;
		}
		for(int i=index; i<history.length(); i++) {
			JSONObject hst = history.getJSONObject(i);
			rs.put(new JSONObject(hst, hst.keys()));
		}
		return rs;
	}

	synchronized public void x_put(String key, String value) throws JSONException 
	{
		st.put(key, value);
	}

	synchronized public void x_put(String key, boolean value) throws JSONException 
	{
		st.put(key, value);
	}

	synchronized public void x_put(String key, double value) throws JSONException 
	{
		st.put(key, value);
	}

	synchronized public void x_put(String key, int value) throws JSONException 
	{
		st.put(key, value);
	}

	synchronized public String x_getString(String key) throws JSONException 
	{
		return st.getString(key);
	}
	
	synchronized public boolean x_getBoolean(String key) throws JSONException 
	{
		return st.getBoolean(key);
	}
	
	synchronized public int x_getInt(String key) throws JSONException 
	{
		return st.getInt(key);
	}
	
	synchronized public double x_getDouble(String key) throws JSONException
	{
		return st.getDouble(key);
	}
	
};
