package au.n800s.robo.brain;

import android.content.Context;
import au.n800s.track.common.DbMsg;

public class LifeSupportThread implements Runnable, TextToSpeech.OnInitListener {

	private Context context;

	boolean isRunning=false;

	private TextToSpeech mTts;

	BaseModel model;

	LifeSupportThread(Context context, BaseModel model) {
		this.context=context;
		this.model = model;
		isRunning=true;
		mTts = new TextToSpeech(this, this);
	}

	public void scream(String text)
	{
		mTts.speak(text, TextToSpeech.QUEUE_ADD, null);
	}

	protected void test_enviroment()
	{
		int etime = model.estimateEnergy();
		if(etime < 300) {
			//scream help if less than 5 minutes
			scream("Help. I need food");
			//search for food
			search_food();
		} else if (etime < 600) {
			//start searching for food if less than 10 minutes
			search_food();
		}
	}

	protected void search_food()
	{
		
	}

	public void run() {

		try {

			while (isRunning)
			{
				test_enviroment();
			}
		} catch (Exception ex) {
			DbMsg.e("doInBackground Exception", ex);
		} finally {
	        if (mTts != null) {
	            mTts.stop();
	            mTts.shutdown();
	        }
		}
		DbMsg.i("Life Support Thread finished");
	}

	void stop() {
		isRunning=false;
	}

	    public void onInit(int status) {
	        // status can be either TextToSpeech.SUCCESS or TextToSpeech.ERROR.
	        if (status == TextToSpeech.SUCCESS) {
	            // Set preferred language to US english.
	            // Note that a language may not be available, and the result will indicate this.
	            int result = mTts.setLanguage(Locale.US);
	            // Try this someday for some interesting results.
	            // int result mTts.setLanguage(Locale.FRANCE);
	            if (result == TextToSpeech.LANG_MISSING_DATA ||
	                result == TextToSpeech.LANG_NOT_SUPPORTED) {
	               // Lanuage data is missing or the language is not supported.
	                DbMsg.e("Language is not available.");
	            } else {
	                // Check the documentation for other possible result codes.
	                // For example, the language may be available for the locale,
	                // but not for the specified country and variant.

	                // The TTS engine has been successfully initialized.
	                // Allow the user to press the button for the app to speak again.
	                // Greet the user.
//	            	mTts.setPitch(30);
			    	mTts.speak("My name is robo.", TextToSpeech.QUEUE_ADD, null);
	            }
	        } else {
	            // Initialization failed.
	            DbMsg.e("Could not initialize TextToSpeech.");
	        }
	    }

};

