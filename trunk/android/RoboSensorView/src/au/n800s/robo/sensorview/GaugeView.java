package au.n800s.robo.sensorview;

import java.util.ArrayList;
import java.util.List;

import java.lang.Math;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Paint.Align;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

public class GaugeView extends View {

	Paint paint = new Paint();
	Float angle;  // View to draw a compass

	public GaugeView(Context context)
	{
		super(context);
		paint.setColor(0xff00ff00);
		paint.setStyle(Style.STROKE);
		paint.setStrokeWidth(2);
		paint.setAntiAlias(true);
	};

	protected void onDraw(Canvas canvas)
	{
		int width = getWidth();
		int height = getHeight();
		int centerx = width/2;
		int centery = height/2;
		int radius = Math.min(height, width)/2;
		canvas.drawCircle(centerx, centery, radius, paint);
		paint.setTextAlign(Align.CENTER);
		canvas.drawText("N", centerx, centery - radius - 10, paint);
		paint.setTextAlign(Align.CENTER);
		canvas.drawText("S", centerx, centery + radius + 10, paint);
//		canvas.drawLine(centerx, 0, centerx, height, paint);
//		canvas.drawLine(0, centery, width, centery, paint);
		// Rotate the canvas with the azimut     
		if (angle != null)
			canvas.rotate(-angle * 360 / (2 * 3.14159f), centerx, centery);
		paint.setColor(0xff0000ff);
		canvas.drawLine(centerx, centery, centerx, centery - radius, paint);
		paint.setColor(0xff00ff00);
	}

	public void setAngle(Float angle)
	{
		this.angle = angle;
		invalidate();
	}

}
