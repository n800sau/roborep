#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<fcntl.h>
#include<pthread.h>

int num_frame=0;

int main()
{
	time_t start,end,exe_time;
	time(&start);

	pthread_t thrd;
	int thrd_f;

	int low_t = 25, high_t = 150, gaussian_kern = 5 , count = 0, kern_size = 3,frame_n=0,occur=0;
	float deviation = 1.41;
	char first = 0;
	int i=0,ret=0,ret_cnt=0,sent=0;
	CvFont font;
	double hScale=1.0;
	double vScale=1.0;
	int    lineWidth=1;
	char buf[3];
	int high=0,low=0;
	CvRect rect;
	int *params=0;
	int frame_count=0;
	char name[]="imgs/img",ext[]=".jpg";
	char full_name[17];

	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth,8);
	CvCapture *ip = cvCaptureFromFile("ipmovie.MPG");
	if(ip == NULL)
	{
		printf("Cant catch any frame\n");
		return 1;
	}
	printf("Capture from file successfull\n");
	sleep(1);
	IplImage *frame = cvQueryFrame(ip);
	IplImage *canny_img = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
	IplImage *gray_img = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
	IplImage *moving_ave = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_32F, 1);
	IplImage *tmp, *diff;

	const char *outfile = "opmovie_new.avi";
	CvVideoWriter *opmovie =  cvCreateVideoWriter(outfile, CV_FOURCC('P', 'I', 'M', '1'), 25, cvSize(frame->width, frame->height),1);

	while(1)
	{
		frame = cvQueryFrame(ip);
		if(frame == NULL)
		{
			printf("Unable to query frame, breaking...after %d\n",count);
			break;
		}

		rect = cvRect (300,0,300,300);
		cvRectangle(frame, cvPoint(300,0), cvPoint (600,300), cvScalar(255,255,0,0),1, 8, 0 );
		cvSetImageROI(frame, rect);
		cvSetImageROI(gray_img, rect);
		cvSetImageROI(canny_img, rect);
		cvSetImageROI(moving_ave, rect);

		cvCvtColor(frame, gray_img, CV_RGB2GRAY);
		cvSmooth(gray_img, gray_img, CV_GAUSSIAN, gaussian_kern, 0, deviation, 0);
		cvCanny(gray_img, canny_img, low_t, high_t, kern_size);

		if(first == 0)
		{
			diff = cvCloneImage(canny_img);
			tmp = cvCloneImage(canny_img);
			cvConvertScale(canny_img, moving_ave, 1, 0);
			printf("First scale conversion done\n");
			first = 1;
		}

		else
			cvRunningAvg(canny_img, moving_ave, 5, NULL);

		cvConvertScale(moving_ave, tmp, 1, 0);
		cvAbsDiff(canny_img, tmp, diff);

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* contour = 0;

		cvFindContours(canny_img, storage, &contour, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		for(;contour!=0;contour = contour->h_next)
		{
			cvDrawContours(frame, contour, cvScalar(125,15,80,150), cvScalarAll(0), 2, 1, 8, cvPoint(0,0));
			i++;
		}
		sprintf(buf,"%d",i);
		if(i>20 && frame_n==0)
		{
			++high;
			frame_n=count;
		}
		if(high > 0 && count < frame_n + 6)
		{
			if(i<75)
			{
				++low;
			}
		}
		else
		{
			high = 0;
			frame_n = 0;
		}

		if(low > 3)
		{
			++occur;
		}

		if(occur>10)
		{
			printf("Fast motion Detected\n");
			low = 0;
			high = 0;
			occur=0;
			frame_n=0;
		}
		sprintf(full_name,"%s%d%s",name,frame_count,ext);

		cvResetImageROI(frame);
		cvResetImageROI(gray_img);
		cvResetImageROI(canny_img);
		cvResetImageROI(moving_ave);

		ret = cvSaveImage(full_name, frame, params);

		if(!ret)
		{
			frame_count=0;
		}
		else
			++frame_count;

		i=0;
		count++;
	}
	printf("%d frames \n", count);
	cvReleaseImage(&tmp);
	cvReleaseImage(&diff);
	cvReleaseImage(&gray_img);
	cvReleaseImage(&moving_ave);
	cvReleaseImage(&canny_img);
	cvReleaseCapture(&ip);
	cvReleaseVideoWriter(&opmovie);
	time(&end);
	exe_time=end-start;
	printf("Total time for execution %d\n",(int)exe_time);
	pthread_join(thrd,NULL);
	return 0;
}
