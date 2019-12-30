package au.n800s.robo.common;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.app.AlertDialog.Builder;
import android.content.Context;
import android.graphics.Bitmap;
import android.net.ConnectivityManager;
import android.os.Environment;
import android.view.WindowManager;
import android.widget.Toast;

public class Utils {

	final static String SAVE_PATH  =Environment.getExternalStorageDirectory().getAbsolutePath()+"/robotarr";

	public static boolean isInternetOn(Context mycontext) {

		ConnectivityManager connec =  (ConnectivityManager) mycontext.getSystemService(Context.CONNECTIVITY_SERVICE);

		if ( 	connec.getNetworkInfo(0).getState() == android.net.NetworkInfo.State.CONNECTED ||
				connec.getNetworkInfo(0).getState() == android.net.NetworkInfo.State.CONNECTING ||
				connec.getNetworkInfo(1).getState() == android.net.NetworkInfo.State.CONNECTING ||
				connec.getNetworkInfo(1).getState() == android.net.NetworkInfo.State.CONNECTED ) {

			return true;
		} else if ( connec.getNetworkInfo(0).getState() == android.net.NetworkInfo.State.DISCONNECTED ||  connec.getNetworkInfo(1).getState() == android.net.NetworkInfo.State.DISCONNECTED  ) {
			return false;
		}
		return false;
	}

	public static String[] shuffle(Object[] arr) {
		List<Object> list = Arrays.asList(arr);
		Collections.shuffle(list);
		return (String[]) list.toArray();
	}


	public static int getRandomInt(int maxvalue) {
		Random randomGenerator = new Random();
		return randomGenerator.nextInt(maxvalue);
	}
	public static int getRandomInt(int maxvalue, int minvalue) {
		Random randomGenerator = new Random();
		return randomGenerator.nextInt(maxvalue-minvalue)+minvalue;
	}


	public static int getWidth(Context mycontext){
		return ((WindowManager) mycontext.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay().getWidth();
	}
	public static int getHeight(Context mycontext){
		return ((WindowManager) mycontext.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay().getHeight();	
	}

	public static void hint(final Context mycontext, final String s) {
		Toast toast=Toast.makeText(mycontext, s, Toast.LENGTH_SHORT);
		toast.show();
	}

	public static void alert(String sHead, String string, Context ctx) {
		try {
			Builder builder = new AlertDialog.Builder(ctx);
			builder.setTitle(sHead); 
			builder.setMessage(string);
			builder.setPositiveButton("ok", null);
			builder.show();
		} catch (Exception ex) {

		} 
	}

	public static InputStream openFileFromAssets(String spath, Context mycontext) {
		try {
			InputStream is = mycontext.getResources().getAssets().open(spath);
			return is;
		} catch (Exception ex) {
			return null;
		}
	}
	public static InputStream openFileFromSD(String spath, Context mycontext) {
		try {
			return loadBitmapFromSD(mycontext, spath);
		} catch (Exception ex) {
			return null;
		}
	}


	private static ProgressDialog dialog;
	public static void showLoaderDialog(String sHead, String sMess, Context ctx) {
		dialog =ProgressDialog.show(ctx, sHead, sMess, true, true);
		dialog.show();
	}
	public static void hideLoaderDialog() {
		if (dialog!=null) dialog.dismiss();
	}

	static boolean isSDCardPrepared() {
		String state = android.os.Environment.getExternalStorageState();
		if (!state.equals(android.os.Environment.MEDIA_MOUNTED)) return false; else return true;
	}

	public static void saveBitmapToSD(Context mycontext, Bitmap bmp, String filename) {
		if (isSDCardPrepared()) {
			File directory = new File(SAVE_PATH);
			directory.mkdirs();

			ByteArrayOutputStream bytes = new ByteArrayOutputStream();
			bmp.compress(Bitmap.CompressFormat.JPEG, 100, bytes);

			File f = new File(SAVE_PATH+File.separator+filename);

			try {
				f.createNewFile();

				FileOutputStream fo = new FileOutputStream(f);
				fo.write(bytes.toByteArray());
			} catch (Exception ex) {
				DbMsg.e("ex saveBitmapToSD", ex);
			}

		} else {
			Utils.alert("Figyelem!", "Hiba! K�rem helyezze be az SD k�rty�t �s h�zza ki az usb k�belt!", mycontext);		
		}
	}
	public static InputStream loadBitmapFromSD(Context mycontext, String filename) {
		if (isSDCardPrepared()) {

			File f = new File(SAVE_PATH+File.separator+filename);

			try {
				if (f.exists()) return new FileInputStream(f);
			} catch (Exception ex) {
				DbMsg.e("ex loadBitmapFromSD", ex);
			}

		} else {
			Utils.alert("Figyelem!", "Hiba! K�rem helyezze be az SD k�rty�t �s h�zza ki az usb k�belt!", mycontext);		
		}

		return null;
	}

	public static boolean isBitmapExistOnSD(String filename) {
		return new File(SAVE_PATH+File.separator+filename).exists();
	}

	public static String getBitmapPath(String filename) {
		return SAVE_PATH+File.separator+filename;
	}


}
