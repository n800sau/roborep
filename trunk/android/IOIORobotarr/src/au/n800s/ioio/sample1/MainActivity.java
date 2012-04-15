package au.n800s.ioio.sample1;

import ioio.lib.api.exception.ConnectionLostException;

import java.io.InputStream;

import java.io.OutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.Date;
import org.json.JSONException;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.content.Context;
import android.widget.TextView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ToggleButton;
import android.view.View.OnClickListener;
import android.view.View;
import android.speech.tts.TextToSpeech;
import android.hardware.SensorListener;
import android.hardware.SensorManager;

/**
 * This is the main activity of the HelloIOIOPower example application.
 * 
 * It displays a toggle button on the screen, which enables control of the
 * on-board LED, as well as a text message that shows whether the IOIO is
 * connected.
 * 
 * Compared to the HelloIOIO example, this example does not use the
 * AbstractIOIOActivity utility class, thus has finer control of thread creation
 * and IOIO-connection process. For a simpler use cases, see the HelloIOIO
 * example.
 */
public class MainActivity extends Activity implements TextToSpeech.OnInitListener {
	
	CommandQueue queue;
	/** The thread that interacts with the IOIO. */
	private IOIOThread ioio_thread_;

	private UDPThread udp_thread_;
	
	private RobotState rstate;

	private Handler mHandler;
	
	private TextToSpeech mTts;

	SensorManager sensorManager;

	Date t_imhungry;
	
	/**
	 * Called when the activity is first created. Here we normally initialize
	 * our GUI.
	 */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		queue = new CommandQueue();
		try {
			rstate = new RobotState();
		} catch(JSONException e) {
			DbMsg.e("onCreate", e);
		}
		t_imhungry = new Date();
		mTts = new TextToSpeech(this, this);
		mHandler = new Handler();
		((Button)findViewById(R.id.button)).setOnClickListener(toggleLed);
		((Button)findViewById(R.id.B_Say)).setOnClickListener(sayText);
		sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
	}

    @Override
    public void onDestroy() {
        // Don't forget to shutdown!
        if (mTts != null) {
            mTts.stop();
            mTts.shutdown();
        }

        super.onDestroy();
    }

	private void setLabel(String msg, int labelId)
	{
		((TextView)findViewById(labelId)).setText(msg);
		
	}
	
	private Runnable mUpdateStateTask = new Runnable() 
	{
		   public void run()
		   {
		     try {
					setLabel(rstate.x_getString("error"), R.id.TV_Error);
					String msg;
					if(rstate.x_getBoolean("connection")) {
						msg = getString(R.string.ioio_connected);	
						setLabel(rstate.x_getString("version"), R.id.TV_Version);
						setLabel(String.valueOf(rstate.x_getInt("battery")), R.id.TV_Battery);
						DbMsg.i(t_imhungry.toString());
						if (rstate.x_getInt("battery") < 4000 && (t_imhungry.getTime() + 30000) < new Date().getTime()) {
							t_imhungry = new Date();
					    	mTts.speak("I'm hungry", TextToSpeech.QUEUE_ADD, null);							
						}
					} else {
						msg = getString(R.string.wait_ioio);
					}
					((TextView)findViewById(R.id.title)).setText(msg);
		     } catch(JSONException e) {
		    	DbMsg.e("mUpdateStateTask", e);		    	 
		     } finally {
				mHandler.postDelayed(this, 1000);
		     }
		   }
	};
	
		// Create an anonymous implementation of OnClickListener
		private OnClickListener toggleLed = new OnClickListener()
		{
		    public void onClick(View v)
		    {
		    	try {
				DbMsg.i(String.valueOf(((ToggleButton)findViewById(R.id.button)).isChecked()));
				rstate.x_put("led", ((ToggleButton)findViewById(R.id.button)).isChecked());
		    	} catch(JSONException e) {
		    		DbMsg.e("toggleLed", e);
		    	}
		    }
		};

		private OnClickListener sayText = new OnClickListener() {
		    public void onClick(View v) {
		    	String text = ((EditText)findViewById(R.id.ET_Say)).getText().toString();
		    	mTts.speak(text, TextToSpeech.QUEUE_FLUSH, null);
		    }
		};

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
	            	((Button)findViewById(R.id.B_Say)).setEnabled(true);
	                // Greet the user.
//	            	mTts.setPitch(30);
			    	mTts.speak("My name is robotarr.", TextToSpeech.QUEUE_ADD, null);
	            }
	        } else {
	            // Initialization failed.
	            DbMsg.e("Could not initialize TextToSpeech.");
	        }
	    }

	    
	    private void updateOrientation(float _roll, float _pitch, float _heading) throws JSONException 
	    {
	          rstate.x_put("heading", _heading);
	          rstate.x_put("pitch", _pitch);
	          rstate.x_put("roll", _roll);
        }

	    private void updateAcceleration(float _Gx, float _Gy, float _Gz) throws JSONException 
	    {
	          rstate.x_put("Gx", _Gx);
	          rstate.x_put("Gy", _Gy);
	          rstate.x_put("Gz", _Gz);
        }

	    private void updateMagneticField(float _Lx, float _Ly, float _Lz) throws JSONException 
	    {
	          rstate.x_put("Lx", _Lx);
	          rstate.x_put("Ly", _Ly);
	          rstate.x_put("Lz", _Lz);
        }


	/**
	 * Called when the application is resumed (also when first started). Here is
	 * where we'll create our IOIO thread.
	 */
	@Override
	protected void onResume() {
		super.onResume();
		try {
			ioio_thread_ = new IOIOThread(queue, rstate);
		ioio_thread_.start();
		udp_thread_ = new UDPThread(queue, rstate);
		udp_thread_.start();
		mHandler.postDelayed(mUpdateStateTask, 1000);
		sensorManager.registerListener(sensorListener, SensorManager.SENSOR_ORIENTATION, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(sensorListener, SensorManager.SENSOR_ACCELEROMETER, SensorManager.SENSOR_DELAY_FASTEST);
		sensorManager.registerListener(sensorListener, SensorManager.SENSOR_MAGNETIC_FIELD, SensorManager.SENSOR_DELAY_FASTEST);
		} catch (ConnectionLostException ex) {
			DbMsg.e("onResume", ex);
		}
	}

	/**
	 * Called when the application is paused. We want to disconnect with the
	 * IOIO at this point, as the user is no longer interacting with our
	 * application.
	 */
	@Override
	protected void onPause() {
		super.onPause();
		mHandler.removeCallbacks(mUpdateStateTask);
		ioio_thread_.abort();
		udp_thread_.abort();
		try {
			ioio_thread_.join();
			udp_thread_.join();
		} catch (InterruptedException e) {
		}
		sensorManager.unregisterListener(sensorListener);
	}

	private final SensorListener sensorListener = new SensorListener() {

        public void onSensorChanged(int sensor, float[] values) 
        {
        	try {
				switch(sensor) {
					case SensorManager.SENSOR_ORIENTATION:
		        		updateOrientation(values[SensorManager.DATA_Z], values[SensorManager.DATA_Y], values[SensorManager.DATA_X]);
						break;
					case SensorManager.SENSOR_ACCELEROMETER:
		        		updateAcceleration(values[SensorManager.DATA_Z], values[SensorManager.DATA_Y], values[SensorManager.DATA_X]);
						break;
					case SensorManager.SENSOR_MAGNETIC_FIELD:
		        		updateMagneticField(values[SensorManager.DATA_Z], values[SensorManager.DATA_Y], values[SensorManager.DATA_X]);
						break;
				}
        	} catch(JSONException e) {
        		DbMsg.e("Sensor error:", e);
        	}
        }

        public void onAccuracyChanged(int sensor, int accuracy) {}

    };
}