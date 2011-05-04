#include "global.h"
#include "slam.h"
#include "bot_ardrone.h"

#include "slam_matlab.h"

#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;


slam::slam()
{
	frame = NULL;
	CV_ready = false;
	prev_frame_descriptors = NULL;

	last_loc[0] = 300;
	last_loc[1] = 300;

	initial_height = -1;

	matlab = new slam_matlab();

	/*
    IplImage* img1 = cvLoadImage("puzzel1.jpg");
    IplImage* img2 = cvLoadImage("puzzel2.jpg");
	IplImage* img3 = cvLoadImage("puzzel5.jpg");

	process_frame(img1);

	cvReleaseImage(&img1);

	process_frame(img2);

	cvReleaseImage(&img2);

	process_frame(img3);
	*/
}


slam::~slam()
{
	cvDestroyWindow("Image:");
}


void slam::run()
{
	DWORD ThreadID;
	h = CreateThread( NULL, 0, process_thread, (void*) this, 0, &ThreadID);
}


static DWORD WINAPI process_thread(void* Param)
{
	slam* This = (slam*) Param; 
	SLAMQUEUE *q = &This->slam_queue;

	This->slam_queue_pushed = CreateEvent(NULL, false, false, (LPTSTR) "SLAM_QUEUE_PUSHED");
	This->slam_queue_empty = CreateEvent(NULL, false, false, (LPTSTR) "SLAM_QUEUE_EMPTY");

	slam_queue_item *item;


	while (!exit_dataset_collector)
	{
		if (q->empty())
		{
			SetEvent(This->slam_queue_empty);
			WaitForSingleObject(This->slam_queue_pushed, INFINITE);
		}

		if (q->empty())
		{
			//printf("queue is empty, while is should not!\n");
			continue;
		}

		item = &q->front();
		q->pop();

		switch (item->type)
		{
			case CONTROL:
				break;
			
			case MEASUREMENT:
				//This->process_measurement( (bot_ardrone_measurement*) item->object );
				break;
			
			case FRAME:
				This->process_frame( (bot_ardrone_frame*) item->object );
				break;
		}
	}

	//This->matlab->display();

	// keep Matlab window alive
	Sleep(99999);

	return 1;
}


void slam::init_CV()
{
	CV_ready = true;

	frame_counter = 0;

	fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);

	de = new SurfDescriptorExtractor();

	canvas = cvCreateImage( cvSize(800,800), 8, 3 );

	prev_frame_h = Mat::eye(3, 3, CV_64F);
	double* h_data = (double*) prev_frame_h.data;
	h_data[2] = 300.0;
	h_data[5] = 300.0;

	canvas_mask = Mat::zeros(800, 800, CV_8UC1);

	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
}

void slam::process_frame(bot_ardrone_frame *f)
{
	if (!CV_ready)
		init_CV();

	if (frame == NULL)
	{
		unsigned short w, h;

		memcpy_s(&w, 2, &f->data[0], 2);
		memcpy_s(&h, 2, &f->data[2], 2);

		w = htons(w);
		h = htons(h);

		frame = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
		gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	}

	frame->imageData = &f->data[4];

	// frames from the real ardrone are received in RGB order instead of BGR
	if (!f->usarsim)
		cvCvtColor( frame, frame, CV_RGB2BGR );

	vector<cv::KeyPoint> keypoints;

	find_features(frame, keypoints, false);


	/* plot keypoints */
	cvCvtColor(frame, gray, CV_RGB2GRAY);


	/* match with previous frame */
	Mat descriptors;
    de->compute(gray, keypoints, descriptors);

	if (frame_counter > 0)
	{
		double ransacReprojThreshold = 3.0;

		/* new method */
		/*
		prev_frame_keypoints.clear();
		prev_frame_descriptors.empty();

		set_canvas_mask();
		find_features(canvas, prev_frame_keypoints, true);
		IplImage *gray2 = cvCreateImage(cvSize(800, 800), IPL_DEPTH_8U, 1);
		cvCvtColor(canvas, gray2, CV_RGB2GRAY);
		de->compute(gray2, prev_frame_keypoints, prev_frame_descriptors);
		*/
		/*****/


		if (keypoints.size() < 20)
		{
			printf("Not enough features found: dropping frame\n");
			return;
		}

		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		if (matches.size() < 6)
			return;

		/* calculate transformation (RANSAC) */
		vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
		for( size_t i = 0; i < matches.size(); i++ )
		{
			queryIdxs[i] = matches[i].queryIdx;
			trainIdxs[i] = matches[i].trainIdx;
		}
	

		vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);

		vector<Point2f> points2;
		KeyPoint::convert(prev_frame_keypoints, points2, trainIdxs);

		Mat homography = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );


		dumpMatrix(homography);


		/* filter method #1: count number of outliers */
		int outliers = 0;

        Mat points3t; perspectiveTransform(Mat(points1), points3t, homography);
        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
            if( norm(points2[i1] - points3t.at<Point2f>((int)i1,0)) > 4 ) // inlier
               outliers++;
        }

		double outliers_percentage =  ((double)outliers / (double)points1.size()) * 100.0;

		//printf("percentage of outers: %f \n", outliers_percentage);

		if (outliers_percentage > 85.0)
		{
			printf("dropped frame based on outliers\n");
			return;
		}

		Mat absolute_homography = prev_frame_h * homography;
		//Mat absolute_homography = homography;


		/* filter method #2: relative change */
		Mat rel_change = prev_frame_h / absolute_homography;
		CvScalar rel_change_avg = mean(rel_change);

		printf("rel change: %f\n", rel_change_avg.val[0]);

		if (abs(rel_change_avg.val[0]) > 3.0)
		{
			printf("dropped frame based on relative changes\n");
			return;
		}

		//prev_frame_h *= homography;
		prev_frame_h = absolute_homography;

		//printf("current pos: %i, %i\n", last_loc[0], last_loc[1]);
	}


	/* get center position */
	double center[] = { (double)frame->width * 0.5, (double)frame->height * 0.5, 0.0 };
	Mat point_center = Mat(1, 3, CV_64F, center);

	Mat tc = point_center * prev_frame_h;

	double *tc_data = (double*) tc.data;
	double *h_data = (double*) prev_frame_h.data;

	last_loc[0] = int(tc_data[0] + h_data[2]);
	last_loc[1] = int(tc_data[1] + h_data[5]);


	/* draw image */
	CvMat CvHomography = prev_frame_h;
	cvWarpPerspective(frame, canvas, &CvHomography, CV_INTER_LINEAR);


	/* store current frame as previous frame */
	prev_frame_keypoints = keypoints;
	prev_frame_descriptors = descriptors;

	frame_counter++;

	imshow("Image:", canvas);
	cvWaitKey(4);
}


void slam::process_frame(IplImage *i)
{
	bot_ardrone_frame *f1 = new bot_ardrone_frame;

	int datasize = i->width * i->height * 3;
	char data[999999];

	unsigned short w, h;
	w = htons(i->width);
	h = htons(i->height);

	memcpy(&data[0], &w, 2);
	memcpy(&data[2], &h, 2);
	memcpy(&data[4], i->imageData, datasize);

	f1->data = data;

	process_frame(f1);
}


void slam::process_measurement(bot_ardrone_measurement *m)
{
	// initial height
	if (initial_height == -1)
		initial_height = m->altitude;

	elevation = initial_height - m->altitude;
	double rel_elevation = (double)elevation / (double)initial_height;

	//printf("elevation: %i, rel_elevation: %f\n", elevation, rel_elevation);

	if (abs(rel_elevation) > 0.2)
	{
		printf("obstacle found: apply mask\n");
	}

	int t[3] = {last_loc[0], last_loc[1], elevation};

	matlab->add_elevation_map_tuple((int*) t);
}


void slam::find_features(IplImage *img, vector<cv::KeyPoint> &v, bool use_mask)
{
	//double tt = (double)cvGetTickCount();

	if (use_mask)
		fd->detect(img, v, canvas_mask);
	else
		fd->detect(img, v);

	printf("found %i features\n", v.size());

	//tt = (double)cvGetTickCount() - tt;
	//printf( "Extraction time = %gms\n", tt/(cvGetTickFrequency()*1000.));
}


void slam::set_canvas_mask()
{
	int minX, maxX, minY, maxY;

	minX = max(0, last_loc[0] - 100);
	minY = max(0, last_loc[1] - 100);

	maxX = min(800, minX + 200);
	maxY = min(800, minY + 200);

	//cvmSetZero(&canvas_mask);
	canvas_mask = Scalar(0);

	for(int x=minX; x<maxX; x++)
	{
		for(int y=minY; y<maxY; y++)
		{
			canvas_mask.at<uint8_t>(y,x) = 1;
		}
	}
}

void slam::PrintMat(CvMat *A)
{
int i, j;
for (i = 0; i < A->rows; i++)
{
printf("\n"); 
switch (CV_MAT_DEPTH(A->type))
{
case CV_32F:
case CV_64F:
for (j = 0; j < A->cols; j++)
printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
break;
case CV_8U:
case CV_16U:
for(j = 0; j < A->cols; j++)
printf ("%6d",(int)cvGetReal2D(A, i, j));
break;
default:
break;
}
}
printf("\n");
}

void slam::dumpMatrix(const Mat &mat) { 
    const int t = mat.type(); 
    for (int i = 0; i < mat.rows; i++) { 
        for (int j = 0; j < mat.cols; j++) { 
            switch (t) { 
            case CV_32F: 
                printf("%6.4f, ", mat.at<float> (i, j)); 
                break; 
            case CV_64F: 
                printf("%6.4f, ", mat.at<double> (i, j)); 
                break; 
            } 
        } 
        printf("\n"); 
    } 
} 