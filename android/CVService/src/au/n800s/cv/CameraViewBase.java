package au.n800s.cv;

import java.io.IOException;
import java.util.List;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import au.n800s.robo.common.DbMsg;

public abstract class CameraViewBase extends SurfaceView implements SurfaceHolder.Callback, Runnable {

    private Camera              mCamera;
    private SurfaceHolder       mHolder;
    private int                 mFrameWidth;
    private int                 mFrameHeight;
    private byte[]              mFrame;
    private boolean             mThreadRun;
    private byte[]              mBuffer;


    public CameraViewBase(Context context) {
        super(context);
        mHolder = getHolder();
        mHolder.addCallback(this);
        DbMsg.i("Instantiated new " + this.getClass());
    }

    public int getFrameWidth() {
        return mFrameWidth;
    }

    public int getFrameHeight() {
        return mFrameHeight;
    }

    public void setPreview() throws IOException {
       	mCamera.setPreviewDisplay(null);
	}

    public void surfaceChanged(SurfaceHolder _holder, int format, int width, int height) {
        DbMsg.i("surfaceCreated");
        if (mCamera != null) {
            Camera.Parameters params = mCamera.getParameters();
            List<Camera.Size> sizes = params.getSupportedPreviewSizes();
            mFrameWidth = width;
            mFrameHeight = height;

            // selecting optimal camera preview size
            {
                int  minDiff = Integer.MAX_VALUE;
                for (Camera.Size size : sizes) {
                    if (Math.abs(size.height - height) < minDiff) {
                        mFrameWidth = size.width;
                        mFrameHeight = size.height;
                        minDiff = Math.abs(size.height - height);
                    }
                }
            }

            params.setPreviewSize(getFrameWidth(), getFrameHeight());

            List<String> FocusModes = params.getSupportedFocusModes();
            if (FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO))
            {
            	params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
            }

            mCamera.setParameters(params);

            /* Now allocate the buffer */
            params = mCamera.getParameters();
            int size = params.getPreviewSize().width * params.getPreviewSize().height;
            size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
            mBuffer = new byte[size];
            /* The buffer where the current frame will be coppied */
            mFrame = new byte [size];
            mCamera.addCallbackBuffer(mBuffer);

			try {
				setPreview();
			} catch (IOException e) {
				DbMsg.e("mCamera.setPreviewDisplay/setPreviewTexture fails: ", e);
			}

            /* Notify that the preview is about to be started and deliver preview size */
            onPreviewStared(params.getPreviewSize().width, params.getPreviewSize().height);

            /* Now we can start a preview */
            mCamera.startPreview();
        }
    }

    public void surfaceCreated(SurfaceHolder holder) {
        DbMsg.i("surfaceCreated");
        mCamera = Camera.open();

        mCamera.setPreviewCallbackWithBuffer(new PreviewCallback() {
            public void onPreviewFrame(byte[] data, Camera camera) {
                synchronized (CameraViewBase.this) {
                    System.arraycopy(data, 0, mFrame, 0, data.length);
                    CameraViewBase.this.notify(); 
                }
                camera.addCallbackBuffer(mBuffer);
            }
        });
                    
        (new Thread(this)).start();
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        DbMsg.i("surfaceDestroyed");
        mThreadRun = false;
        if (mCamera != null) {
            synchronized (this) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);
                mCamera.release();
                mCamera = null;
            }
        }
        onPreviewStopped();
    }

    /* The bitmap returned by this method shall be owned by the child and released in onPreviewStopped() */
    protected abstract Bitmap processFrame(byte[] data);

    /**
     * This method is called when the preview process is beeing started. It is called before the first frame delivered and processFrame is called
     * It is called with the width and height parameters of the preview process. It can be used to prepare the data needed during the frame processing.
     * @param previewWidth - the width of the preview frames that will be delivered via processFrame
     * @param previewHeight - the height of the preview frames that will be delivered via processFrame
     */
    protected abstract void onPreviewStared(int previewWidtd, int previewHeight);

    /**
     * This method is called when preview is stopped. When this method is called the preview stopped and all the processing of frames already completed.
     * If the Bitmap object returned via processFrame is cached - it is a good time to recycle it.
     * Any other resourcses used during the preview can be released.
     */
    protected abstract void onPreviewStopped();

    public void run() {
        mThreadRun = true;
        DbMsg.i("Starting processing thread");
        while (mThreadRun) {
            Bitmap bmp = null;

            synchronized (this) {
                try {
                    this.wait();
                    bmp = processFrame(mFrame);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (bmp != null) {
                Canvas canvas = mHolder.lockCanvas();
                if (canvas != null) {
                    canvas.drawBitmap(bmp, (canvas.getWidth() - getFrameWidth()) / 2, (canvas.getHeight() - getFrameHeight()) / 2, null);
                    mHolder.unlockCanvasAndPost(canvas);
                }
            }
        }
    }
}