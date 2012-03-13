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
		put("error", "");
	}

}
