package au.n800s.cv;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;
import au.n800s.track.common.DbMsg;

public class CameraActivity extends Activity {

    public CameraActivity() {
        DbMsg.i("Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        DbMsg.i("onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(new CameraView(this));
    }
}
