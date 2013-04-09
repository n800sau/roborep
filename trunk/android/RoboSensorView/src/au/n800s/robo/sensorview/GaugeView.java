package au.n800s.robo.sensorview;

import java.util.ArrayList;
import java.util.List;
import android.util.AttributeSet;

import java.lang.Math;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Paint.Align;
import android.graphics.drawable.Drawable;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

public class GaugeView extends View {

	private Paint paint = new Paint();
	private Float angle;  // View to draw a compass in radians
	private Float degrees;  // View to draw a compass in degrees
	private int	 bgcolor;

	public GaugeView(Context context, AttributeSet attrs, int defStyle)
	{
		super(context, attrs, defStyle);
		init();
	}

	public GaugeView(Context context, AttributeSet attrs)
	{
		super(context, attrs);
		init();
	}

	public GaugeView(Context context)
	{
		super(context);
		init();
	}

	private void init()
	{
		paint.setColor(0xff00ff00);
		paint.setStrokeWidth(2);
		paint.setAntiAlias(true);
	}

    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        setMeasuredDimension(100, 130);
    }

	protected void onDraw(Canvas canvas)
	{
		int width = getWidth();
		int height = getHeight();
		paint.setColor(this.bgcolor);
		Log.d("RoboSensorView", String.format("Color:%d", this.bgcolor));
		paint.setStyle(Style.FILL);
        canvas.drawRect(0, 0, width, height, paint);
		paint.setStyle(Style.STROKE);
		paint.setColor(0xff00ff00);
		int centerx = width/2;
		int centery = height/2;
		int radius = Math.min(height, width)/2 - 20;
		canvas.drawCircle(centerx, centery, radius, paint);
		paint.setTextAlign(Align.CENTER);
		canvas.drawText("N", centerx, centery - radius - 10, paint);
		paint.setTextAlign(Align.CENTER);
		canvas.drawText("S", centerx, centery + radius + 10, paint);
//		canvas.drawLine(centerx, 0, centerx, height, paint);
//		canvas.drawLine(0, centery, width, centery, paint);
		// Rotate the canvas with the azimut     
		if (angle != null) {
			paint.setTextAlign(Align.LEFT);
			canvas.drawText("helli", 1, 10, paint);
			canvas.rotate(-angle * 360 / (2 * 3.14159f), centerx, centery);
		} else if (degrees != null) {
			canvas.rotate(degrees, centerx, centery);
		}
		paint.setColor(0xff0000ff);
		canvas.drawLine(centerx, centery, centerx, centery - radius, paint);
		paint.setColor(0xff00ff00);
	}

	public void setBackgroundColor(int clr)
	{
		super.setBackgroundColor(clr);
		this.bgcolor = clr;
		invalidate();
	}

	public void setAngle(Float angle)
	{
		this.degrees = null;
		this.angle = angle;
		invalidate();
	}

	public void setDegrees(Float degrees)
	{
		this.angle = null;
		this.degrees = degrees;
		invalidate();
	}

}
