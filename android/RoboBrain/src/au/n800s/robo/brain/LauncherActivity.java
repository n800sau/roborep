package au.n800s.robo.brain;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

public class LauncherActivity extends Activity {

    private static final String ROBO_DIRECTORY = "Robotarr";
    private static final String CONFIG_FILENAME = "brain.conf";

	private Properties conf = new Properties();

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		FileReader reader = new FileReader(new File(Environment.getExternalStorageDirectory() + "/" + ROBO_DIRECTORY + "/" + CONFIG_FILENAME));
		conf.load(reader);
		reader.close();
		Intent intent = new Intent(this, BrainService.class);
		intent.putExtra("model", conf.getProperty("model", "3pi"));
		startService(intent);
		finish();
	}

}