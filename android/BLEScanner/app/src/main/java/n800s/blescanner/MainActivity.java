package n800s.blescanner;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.Toast;

public class MainActivity extends AppCompatActivity {

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
	}

	@Override
	protected void onResume() {
		super.onResume();
		BluetoothLeScannerCompat scanner = BluetoothLeScannerCompat.getScanner();
		ScanSettings settings = new ScanSettings.Builder()
				.setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).setReportDelay(1000)
				.setUseHardwareBatchingIfSupported(false).build();
		List<ScanFilter> filters = new ArrayList<>();
		filters.add(new ScanFilter.Builder().setServiceUuid(mUuid).build());
		scanner.startScan(filters, settings, scanCallback);
	}

	@Override
	protected void onPause() {
		super.onPause();
		BluetoothLeScannerCompat scanner = BluetoothLeScannerCompat.getScanner();
		scanner.stopScan(scanCallback);
	}



	protected void onDestroy() {
		super.onDestroy();

	}

}
