package au.n800s.ioio.sample1;

import java.util.Calendar;
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;
import java.util.ArrayList;

public class RobotState extends JSONObject {
	
	private ArrayList<JSONObject> history;
	private JSONObject st;
	int history_offset;

	RobotState() throws JSONException
	{
		history_offset = 0;
		history = new ArrayList<JSONObject>();
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
		st.put("index", history.size());
		st.put("timestamp", Calendar.getInstance().getTime().getMinutes());
	}


	synchronized protected void x_pushState() throws JSONException
	{
		st.put("timestamp", Calendar.getInstance().getTime().getMinutes());
		while(history.size() > 10000) {
			history.remove(0);
			history_offset++;
		}
		history.add(st);
		st = new JSONObject(st.toString());
		st.put("index", history.size());
	}

	synchronized public JSONObject x_current_state() throws JSONException
	{
		return new JSONObject(st.toString());
	}

	synchronized public JSONArray x_history(int startIndex) throws JSONException
	{
		JSONArray rs = new JSONArray();
		int index = startIndex - history_offset;
		if(index < 0) {
			index = 0;
		}
		for(int i=index; i<history.size(); i++) {
			JSONObject hst = history.get(i);
			rs.put(new JSONObject(hst.toString()));
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
