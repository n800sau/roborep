package au.n800s.ioio.sample1;

//Object that is located at X,Y
public class WorldPoint {

	public int obstacle = 0;

	public JSONObject asJSON()
	{
		JSONObject rs = new JSONObject();
		rs.put("obstacle", obstacle);
		return rs;
	}

};
