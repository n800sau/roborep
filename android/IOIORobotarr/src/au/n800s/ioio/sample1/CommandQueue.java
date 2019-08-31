package au.n800s.ioio.sample1;

import java.util.ArrayList;

public final class CommandQueue {

	ArrayList<Command> queue;
	
	CommandQueue() {
		queue = new ArrayList<Command>();
	}
	
	synchronized public void addCommand(Command command) {
		queue.add(command);
	}

	synchronized public Command nextCommand() {
		Command rs=null;
		if(queue.size() > 0) {
			rs = queue.get(0);
			queue.remove(0);
		}
		return rs;
	}

}
