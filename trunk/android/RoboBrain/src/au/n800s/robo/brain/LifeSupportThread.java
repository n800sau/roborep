package au.n800s.robo.brain;

import android.content.Context;
import au.n800s.track.common.DbMsg;

public class LifeSupportThread implements Runnable {

	private Context context;

	boolean isRunning=false;

	LifeSupportThread(Context context) {
		this.context=context;
		isRunning=true;
	}

	public void run() {

		try {

			while (isRunning) 
			{
			}
		} catch (Exception ex) {
			DbMsg.e("doInBackground Exception", ex);
		}
		DbMsg.i("Life Support Thread finished");
	}

	void stop() {
		isRunning=false;
	}

};

