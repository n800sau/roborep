package au.n800s.robo.sensorview;

import android.app.Activity;
import android.os.Bundle;

import android.util.Log;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import android.widget.TableLayout;
import android.widget.TableRow;

import android.graphics.Color;

import com.nullwire.trace.ExceptionHandler;

import android.view.View.OnClickListener;
import android.os.Handler;
import android.os.Message;
import android.widget.TextView;
import android.widget.Button;
import android.view.View;

public class RoboSensorView extends Activity implements SensorEventListener, OnClickListener
{
	public static final String SERVERIP = "115.70.59.149";
	public static final Integer SERVERPORT = 7980;
	protected GaugeView gauge1, gauge2, gauge3, gauge4, gauge5;
	private SensorManager mSensorManager;
	protected Sensor accelerometer;
	protected Sensor magnetometer;
	public Handler Handler;
	public TextView text1;
	public Button btn;

	private UDPServer srv;
	private UDPClient cln;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
		ExceptionHandler.register(this);
		ExceptionHandler.register(this, "http://n800s.dyndns.org/android/server.php");
		Log.d("RoboSensorView", "Hello");
		setContentView(R.layout.main);
/*		TableLayout table = new TableLayout(this);
		table.setStretchAllColumns(true);
		table.setShrinkAllColumns(true);
		table.setColumnStretchable(0, true);

		TableRow row1 = new TableRow(this);
		TableRow row2 = new TableRow(this);
		TableRow row3 = new TableRow(this);
*/
		text1=(TextView)findViewById(R.id.textView1);

//		gauge1 = new GaugeView(this);
		gauge1 = (GaugeView)findViewById(R.id.g1);
		gauge1.setBackgroundColor(Color.RED);
		gauge1.invalidate();

		gauge2 = (GaugeView)findViewById(R.id.g2);
		gauge2.setBackgroundColor(Color.MAGENTA);
		gauge2.invalidate();

//		gauge3 = new GaugeView(this);
		gauge3 = (GaugeView)findViewById(R.id.g3);
		gauge3.setBackgroundColor(Color.YELLOW);
		gauge3.invalidate();
//		gauge.requestFocus();
//		gauge.setAngle(Float.valueOf(45));

//		gauge4 = new GaugeView(this);
//		gauge4.setBackgroundColor(Color.CYAN);
//		gauge4.invalidate();

//		gauge5 = new GaugeView(this);
//		gauge5.setBackgroundColor(Color.GREEN);
//		gauge5.invalidate();

/*		row1.addView(gauge1);
		row1.addView(gauge2);
		row2.addView(gauge3);
		row3.addView(gauge4);
		row3.addView(gauge5);
		table.addView(row1);
		table.addView(row2);
		table.addView(row3);
*///		setContentView(table);
//		try {
//		} catch (Exception e) {
//			Log.e("ERROR", "ERROR IN CODE:"+e.toString());
//		}

//        setContentView(gauge);
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

		Handler = new Handler() {
			@Override public void handleMessage(Message msg)
			{
				String text = (String)msg.obj;
				text1.append(text);
			}
		};

		srv = new UDPServer(Handler, SERVERIP, SERVERPORT);
		new Thread(srv).start();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) { }
		cln = new UDPClient(Handler, SERVERIP, SERVERPORT);
		new Thread(cln).start();
		btn = (Button)findViewById(R.id.button1);
		btn.setOnClickListener(this);

    }

  protected void onResume() {
    super.onResume();
    mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
    mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
  }
 
  protected void onPause() {
    super.onPause();
    mSensorManager.unregisterListener(this);
  }
 
  public void onAccuracyChanged(Sensor sensor, int accuracy) {  }
 
  float[] mGravity;
  float[] mGeomagnetic;
  public void onSensorChanged(SensorEvent event) {
    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
      mGravity = event.values;
    if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
      mGeomagnetic = event.values;
    if (mGravity != null && mGeomagnetic != null) {
      float R[] = new float[9];
      float I[] = new float[9];
      boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
      if (success) {
        float orientation[] = new float[3];
        SensorManager.getOrientation(R, orientation);
        gauge1.setAngle(orientation[0]); // orientation contains: azimut, pitch and roll
        gauge2.setAngle(orientation[1]); // orientation contains: azimut, pitch and roll
        gauge3.setAngle(orientation[2]); // orientation contains: azimut, pitch and roll
      }
    }
  }

	public void updatetrack(String s){
		Message msg = new Message();
		String textTochange = s;
		msg.obj = textTochange;
		Handler.sendMessage(msg);
	}

	@Override
	public void onClick(View v)
	{
		cln.start=true;
		srv.start=true;
	}

}
