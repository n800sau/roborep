package au.n800s.robo.sensorview;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class RoboSensorView extends Activity implements SensorEventListener
{

	GaugeView gauge;
  private SensorManager mSensorManager;
  Sensor accelerometer;
  Sensor magnetometer;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
//		try {
//			setContentView(R.layout.main);
//		} catch (Exception e) {
//			Log.e("ERROR", "ERROR IN CODE:"+e.toString());
//		}

		gauge = new GaugeView(this);
        setContentView(gauge);
        gauge.requestFocus();
//		gauge.setAngle(Float.valueOf(45));
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
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
        gauge.setAngle(orientation[0]); // orientation contains: azimut, pitch and roll
      }
    }
  }

}
