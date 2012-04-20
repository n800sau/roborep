package au.n800s.robo.brain;

import android.content.Context;
import au.n800s.track.common.DbMsg;

public class LifeSupportThread implements Runnable {

	private Context context;

	boolean isRunning=false;

	BaseModel model;

	LifeSupportThread(Context context, BaseModel model) {
		this.context=context;
		this.model = model;
		isRunning=true;
	}

	public void run() {

		try {

			while (isRunning)
			{
				int etime = model.estimateEnergy();
				if(etime < 300) {
					//scream help if less than 5 minutes
				} else if (etime < 600) {
					//start searching for food if less than 10 minutes
				}
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

