package au.n800s.ioio.sample1;

import java.util.Calendar;
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;
import java.util.ArrayList;
import java.util.Map;
import java.util.Pair;

//distance in meters
public class RobotState {
	
	static int PWM_COUNT = 5;
	static int IR_COUNT = 5;

	private ArrayList<JSONObject> history;
	private JSONObject st;
	int history_offset;
	private Map<Pair<Integer,Integer>, WorldPoint> map = new Map<Pair<Integer,Integer>, WorldPoint>();
	//location, heading -> pwr (2d vector - location and length)
	//vector -> (x,y, angle) indexed by pwr
	private Map<<Integer, Integer, Integer>, Integer> charger_direction = new Map<Integer, Integer>();

	RobotState() throws JSONException
	{
		//angles in degrees
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
		st.put("Lx", 0);
		st.put("Ly", 0);
		st.put("Lz", 0);
		st.put("current_heading", -1);
		st.put("ir_raw0", 0);
		st.put("ir_raw1", 0);
		st.put("ir_raw2", 0);
		st.put("ir_raw3", 0);
		st.put("ir_raw4", 0);
		st.put("head_angle", 0);
		st.put("head_distance", -1);
		st.put("beacon_pwr", 0);
		st.put("x", 0);
		st.put("y", 0);
		st.put("error", "");
		st.put("index", history.size());
		st.put("timestamp", Calendar.getInstance().getTime().getMinutes());
		DbMsg.i(x_getString("version"));
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
		for(int i=0; i<Math.min(10, history.size()-index); i++) {
			JSONObject hst = history.get(i + index);
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

	synchronized public void x_put(String key, JSONArray value) throws JSONException 
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
	
	synchronized public JSONArray x_JSONArray(String key) throws JSONException
	{
		return st.getJSONArray(key);
	}

	protected Pair<int,int> calc_location(double angle, double distance)
	{
		int x = st.getInt("x"), y = st.getInt("y"), dx, dy;
		angle = st.getInt("current_heading") + angle;
		dx = (int)(Math.sin(Math.toRadians(angle)) * distance);
		dy = (int)(Math.cos(Math.toRadians(angle)) * distance);
		return Pair(x + dx, y + dy);
	}

	protected WorldPoint loc_obj(Pair<int,int> loc)
	{
		return (map.containsKey(loc)) ? map.get(loc) : new WorldPoint()
	}

	synchronized public addPwd(int val)
	{
		charger_direction.set(st.getInt("current_heading"), val);
	}

	synchronized public maxPwrDirection()
	{
		int maxPwr = 0;
		for(int i=0; i< charger_direction.size(); i++) {
			//current location
			//
			if(maxPwr < charger_direction.get(
		}
	}

	synchronized public addDistance(int val)
	{
		Pair<int, int> loc = calc_location(st.getInt("head_angle"), distance);
		WorldPoint pnt = loc_obj(loc);
		pnt.obstacle += 1;
		map.put(loc, pnt);
	}

	synchronized public JSONArray mapArray()
	{
		JSONArray rs = new JSONArray();
		WorldPoint pnt;
		for( Iterator<Pair<Integer,Integer>> keyIter = map.keySet().iterator(); keyIter.hasNext()) {
			jo = new JSONObject();
			pnt = map.get(keyIter.next());
			rs.put(i, pnt.asJSON(););
		}
		return rs;
	}

	//thread calculating current location and speeds
	//spawn calculations??

};
