package n800s.bleservice;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.content.Intent;
import android.content.Context;
import android.util.Log;

public class MainActivity extends AppCompatActivity {

	private static final String TAG = "bleservice.MainActivity";

	private Context ctx = this;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
		startService(new Intent(ctx, BLEService.class));
		Log.d(TAG, "Successfully started");
    }
}
