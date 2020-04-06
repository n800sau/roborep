package au.n800s.ioio.sample1;

import org.json.JSONObject;

public final class Command {

	String name;
	JSONObject params;
	
	Command(String name, JSONObject params)
	{
		this.name = name;
		this.params = params;
	}
}
