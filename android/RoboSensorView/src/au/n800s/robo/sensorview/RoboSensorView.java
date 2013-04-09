package au.n800s.robo.sensorview;

import android.app.Activity;
import android.os.Bundle;

import android.util.Log;

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
import android.preference.PreferenceManager;
import android.content.SharedPreferences;

import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.content.Intent;

public class RoboSensorView extends Activity implements OnClickListener
{
	public static final String SERVERIP = "115.70.59.149";
	public static final String SERVERPORT = "7980";
	public Handler Handler;
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
							cmd.put("interval", 300);
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
			TextView tv;
			int count = jsobjlist.length;
			for (int i = 0; i < count; i++) {
				try {
					((TextView)findViewById(R.id.s_timestamp)).setText(jsobjlist[i].getString("s_timestamp"));

					Time t = new Time();

					t.set(jsobjlist[i].getJSONObject("adxl345.js.obj").getLong("timestamp") * 1000);
					((TextView)findViewById(R.id.adxl345_timestamp)).setText(t.format("%Y.%m.%d %H:%M:%S.%u"));
					((GaugeView)findViewById(R.id.adxl345_xz_heading)).setDegrees((float)jsobjlist[i].getJSONObject("adxl345.js.obj").getDouble("xz_degrees"));
					((GaugeView)findViewById(R.id.adxl345_yz_heading)).setDegrees((float)jsobjlist[i].getJSONObject("adxl345.js.obj").getDouble("yz_degrees"));

					t.set(jsobjlist[i].getJSONObject("hmc5883l.js.obj").getLong("timestamp") * 1000);
					((TextView)findViewById(R.id.hmc5883l_timestamp)).setText(t.format("%Y.%m.%d %H:%M:%S.%u"));
					((GaugeView)findViewById(R.id.hmc5883l_heading)).setDegrees((float)jsobjlist[i].getJSONObject("hmc5883l.js.obj").getDouble("heading_degrees"));

					t.set(jsobjlist[i].getJSONObject("mpu6050.js.obj").getLong("timestamp") * 1000);
					((TextView)findViewById(R.id.mpu6050_timestamp)).setText(t.format("%Y.%m.%d %H:%M:%S.%u"));
					((GaugeView)findViewById(R.id.mpu6050_xz_degrees)).setDegrees((float)jsobjlist[i].getJSONObject("mpu6050.js.obj").getDouble("xz_degrees"));
					((GaugeView)findViewById(R.id.mpu6050_yz_degrees)).setDegrees((float)jsobjlist[i].getJSONObject("mpu6050.js.obj").getDouble("yz_degrees"));

//					int resId = getResources().getIdentifier("adxl345_xz_degrees_timestamp", "id", getPackageName());
//					if(resId > 0) {
//						tv = (TextView)findViewById(resId);
//						tv.set(jsobjlist[i].getJSONObject("adxl345.js.obj").getLong("timestamp") * 1000);
//					}

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
		Log.d(logid, "Hello");
		setContentView(R.layout.main);

/*		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
*/
		Handler = new Handler() {
			@Override public void handleMessage(Message msg)
			{
				String text = (String)msg.obj;
				TextView tv = (TextView)findViewById(R.id.errormsg);
				tv.setText(text);
			}
		};

		b_restart = (Button)findViewById(R.id.b_restart);
		b_restart.setOnClickListener(this);
		onClick(b_restart);

    }

  protected void onResume() {
    super.onResume();
  }
 
  protected void onPause() {
    super.onPause();
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
		try {
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			String server = sharedPref.getString("server", SERVERIP);
			Integer port = Integer.valueOf(sharedPref.getString("port", SERVERPORT));
			Socket socket = new Socket(server, port);
			DataReceiverTask task;
			task = new DataReceiverTask();
			task.execute(socket);
		} catch(Exception e) {
			updatetrack(e.toString());
		}
	}

	public boolean onCreateOptionsMenu(Menu menu)
	{
		super.onCreateOptionsMenu(menu);
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.options, menu);
		return true;
	}

	public boolean onOptionsItemSelected(MenuItem item)
	{
		switch (item.getItemId())
		{
			case R.id.settings_title:
				startActivity(new Intent(this, SettingsActivity.class));
				return true;
			case R.id.exit_title:
				finish();
				return true;
		}
		return false;
	}

}
