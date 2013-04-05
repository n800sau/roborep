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

import java.net.Socket;
import org.json.JSONObject;
import org.json.JSONException;
import android.os.AsyncTask;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStreamReader;
import java.io.IOException;

import java.util.Arrays;
import java.util.List;

import android.text.format.Time;


public class RoboSensorView extends Activity implements SensorEventListener, OnClickListener
{
	public static final String SERVERIP = "115.70.59.149";
	public static final Integer SERVERPORT = 7980;
	protected GaugeView g_adxl345_xz_heading, g_adxl345_yz_heading, g_hmc5883l_heading;
	private SensorManager mSensorManager;
	protected Sensor accelerometer;
	protected Sensor magnetometer;
	public Handler Handler;
	public TextView s_timestamp, s_adxl345_timestamp, s_hmc5883l_timestamp;
	public Button b_restart;
	private DataReceiverTask task;

	public static final String logid = "RoboSensorView";

	private class DataReceiverTask extends AsyncTask<Socket, JSONObject, Boolean> {

		/** The system calls this to perform work in a worker thread and
		* delivers it the parameters given to AsyncTask.execute() */
		protected Boolean doInBackground(Socket... socketargs) {
			List<Socket> sockets = Arrays.asList(socketargs);
			Socket socket;
			int count = sockets.size();
			for (int i = 0; i < count; i++) {
				socket = sockets.get(i);
				if(socket.isConnected()) {
					try {
						PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
						JSONObject cmd = new JSONObject();
						try {
							cmd.put("cmd", "send_full_data");
							cmd.put("interval", 100);
							cmd.put("count", 100);
							out.println(cmd.toString());
						} catch(JSONException e) {
							updatetrack(e.toString());
						}
						out.flush();
					} catch(IOException e) {
						Log.e(logid, e.toString());
					}
				}
			}
			while(!isCancelled() && sockets.size() > 0) {
				count = sockets.size();
				for (int i = 0; i < count; i++) {
					socket = sockets.get(i);
					if(socket.isConnected()) {
						try {
							BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
//							StringBuilder jsin = new StringBuilder();
							while(socket.isConnected() && !isCancelled()) {
								String line = in.readLine();
//								if(line != "EOF") {
//									jsin.append(line + " ");
//								} else {
//									jsin = new StringBuilder();
									try {
										JSONObject jsobj = new JSONObject(line);
//										JSONObject jsobj = new JSONObject(jsin.toString());
										Log.i(logid, line);
										publishProgress(jsobj);
									} catch(Exception e) {
										Log.e(logid, e.toString());
									}
//									break;
//								}
							}
						} catch(IOException e) {
							Log.e(logid, e.toString());
						}
					} else {
						sockets.remove(i);
					}
				}
			}
			return !isCancelled();
		}

		/** The system calls this to perform work in the UI thread and delivers
		* the result from doInBackground() */
		protected void onPostExecute(Boolean success) {
		}

		protected void onProgressUpdate(JSONObject... jsobjlist) {
			int count = jsobjlist.length;
			for (int i = 0; i < count; i++) {
				try {
					updatetrack(jsobjlist[i].getString("s_timestamp"));

					Time t = new Time();

					t.set(jsobjlist[i].getJSONObject("adxl345.js.obj").getLong("timestamp") * 1000);
					s_adxl345_timestamp.setText(t.format("%Y.%m.%d %H:%M:%S.%u"));
					Double v0 = jsobjlist[i].getJSONObject("adxl345.js.obj").getDouble("xz_degrees");
					g_adxl345_xz_heading.setDegrees(v0.floatValue());
					Double v1 = jsobjlist[i].getJSONObject("adxl345.js.obj").getDouble("yz_degrees");
					g_adxl345_yz_heading.setDegrees(v1.floatValue());

					t.set(jsobjlist[i].getJSONObject("hmc5883l.js.obj").getLong("timestamp") * 1000);
					s_hmc5883l_timestamp.setText(t.format("%Y.%m.%d %H:%M:%S.%SS"));
					Double v2 = jsobjlist[i].getJSONObject("hmc5883l.js.obj").getDouble("heading_degrees");
					g_hmc5883l_heading.setDegrees(v2.floatValue());
				} catch(JSONException e) {
					updatetrack(e.toString());
				}
			}
		}

	}

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
		ExceptionHandler.register(this);
		ExceptionHandler.register(this, "http://n800s.dyndns.org/android/server.php");
		Log.d(logid, "Hello");
		setContentView(R.layout.main);

		s_timestamp=(TextView)findViewById(R.id.s_timestamp);

		s_adxl345_timestamp = (TextView)findViewById(R.id.adxl345_timestamp);
		g_adxl345_xz_heading = (GaugeView)findViewById(R.id.adxl345_xz_heading);
		g_adxl345_yz_heading = (GaugeView)findViewById(R.id.adxl345_yz_heading);

		s_hmc5883l_timestamp = (TextView)findViewById(R.id.hmc5883l_timestamp);
		g_hmc5883l_heading = (GaugeView)findViewById(R.id.hmc5883l_heading);

		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

		Handler = new Handler() {
			@Override public void handleMessage(Message msg)
			{
				String text = (String)msg.obj;
				s_timestamp.setText(text);
			}
		};

		try {
			Socket socket = new Socket(SERVERIP, SERVERPORT);
			task = new DataReceiverTask();
			task.execute(socket);
			b_restart = (Button)findViewById(R.id.b_restart);
			b_restart.setOnClickListener(this);
		} catch(Exception e) {
		}

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
//        gauge1.setAngle(orientation[0]); // orientation contains: azimut, pitch and roll
//        gauge2.setAngle(orientation[1]); // orientation contains: azimut, pitch and roll
//        gauge3.setAngle(orientation[2]); // orientation contains: azimut, pitch and roll
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
	}

}
