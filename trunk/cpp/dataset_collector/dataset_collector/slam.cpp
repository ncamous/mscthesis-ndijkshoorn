#include "slam.h"
#include "bot_ardrone.h"

#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


slam::slam()
{
	frame = NULL;
	openCV_init = false;
}


slam::~slam()
{
	cvDestroyWindow("Image:");
}


void slam::init_openCV()
{
	openCV_init = true;

	/*
	SurfFeatureDetector( double hessianThreshold=400., int octaves=3, int octaveLayers=4 );
	*/

	fd = new SurfFeatureDetector(2500., 3, 4);


	/*
	SiftFeatureDetector( const SIFT::DetectorParams& detectorParams=SIFT::DetectorParams(),
                         const SIFT::CommonParams& commonParams=SIFT::CommonParams() );
    SiftFeatureDetector( double threshold, double edgeThreshold,
                         int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
                         int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
                         int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
                         int angleMode=SIFT::CommonParams::FIRST_ANGLE );
	*/

	/*
	fd = new SiftFeatureDetector(
		SIFT::DetectorParams::GET_DEFAULT_THRESHOLD() * 10.0,
		SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD() * 10.0
	);
	*/

	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
}

void slam::process_frame(bot_ardrone_frame *f)
{
	if (!openCV_init)
		init_openCV();


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

	vector<cv::KeyPoint> keypoints;

	find_features(frame, keypoints);


	/* plot keypoints */
	// output image
	Mat out;
	// draw grayscale image
	cvCvtColor(frame, gray, CV_RGB2GRAY);

	drawKeypoints(gray, keypoints, out);

	imshow("Image:", out);
	/**/

	cvWaitKey(2);
}


void slam::find_features(IplImage *img, vector<cv::KeyPoint> &v)
{
	double tt = (double)cvGetTickCount();

	fd->detect(img, v);

	tt = (double)cvGetTickCount() - tt;

	printf( "Extraction time = %gms\n", tt/(cvGetTickFrequency()*1000.));
}