#include "slam.h"
#include "bot_ardrone.h"

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
}


slam::~slam()
{
	cvDestroyWindow("Image:");
}


void slam::init_CV()
{
	CV_ready = true;

	frame_counter = 0;

	/*
	SurfFeatureDetector( double hessianThreshold=400., int octaves=3, int octaveLayers=4 );
	*/

	fd = new SurfFeatureDetector(/*2500.*/ 1500., 3, 4);


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

	de = new SurfDescriptorExtractor();

	//dm = new BruteForceMatcher<Hamming>();

	canvas = cvCreateImage( cvSize(800,800), 8, 3 );

	tmp_xoffset = 300;
	tmp_yoffset = 300;

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

	vector<cv::KeyPoint> keypoints;

	find_features(frame, keypoints);


	/* plot keypoints */
	// output image
	//Mat out;
	// draw grayscale image
	cvCvtColor(frame, gray, CV_RGB2GRAY);

	//drawKeypoints(gray, keypoints, out);


	/* match with previous frame */
	Mat descriptors;
    de->compute(gray, keypoints, descriptors);

	if (frame_counter > 0)
	{
		double ransacReprojThreshold = 3.0;

		//simpleMatching(descriptors, prev_frame_descriptors, matches);
		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		printf("frame 1 has %i keypoints\n", prev_frame_keypoints.size());
		printf("frame 2 has %i keypoints\n", keypoints.size());
		printf("found %i matches\n", matches.size());

		if (matches.size() < 4)
			return;

		/* calculate transformation (RANSAC) */
		double tt = (double)cvGetTickCount();

		vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
		for( size_t i = 0; i < matches.size(); i++ )
		{
			queryIdxs[i] = matches[i].queryIdx;
			trainIdxs[i] = matches[i].trainIdx;
		}
	
		printf("Computing homography (RANSAC)...\n");

		vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);

		vector<Point2f> points2;
		KeyPoint::convert(prev_frame_keypoints, points2, trainIdxs);

		//CvMat CvHomography = *cvCreateMat(3, 3, CV_32F);
		//cvSetIdentity(&CvHomography);
		Mat homography = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );
		CvMat CvHomography = homography;

		//dumpMatrix(homography);


		/*CvPoint2D32f srcQuad[4], dstQuad[4];
		CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

		cvGetPerspectiveTransform(
			srcQuad,
			dstQuad,
			warp_matrix
		);*/

		tt = (double)cvGetTickCount() - tt;

		//printf( "RANASC transformation time = %gms\n", tt/(cvGetTickFrequency()*1000.));


		/* transform current frame */
		int incX, incY;
		incX = (int) cvGetReal2D(&CvHomography, 0, 2);
		incY = (int) cvGetReal2D(&CvHomography, 1, 2);

		if (incX > 40 || incY > 40)
			return;

		tmp_xoffset += incX;
		tmp_yoffset += incY;

		if (
			(tmp_xoffset + frame->width) > 800 ||
			(tmp_yoffset + frame->height) > 800 ||
			tmp_xoffset < 0 ||
			tmp_yoffset < 0)
			return;

		CvHomography.data.fl[2] = CvHomography.data.fl[5] = 0.0f;

		//IplImage *dst;
		//dst = cvCloneImage(frame);
		//dst->origin = frame->origin;
		//cvZero(dst);
		IplImage *dst = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 3);

		cvWarpPerspective(frame, dst, &CvHomography, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

		// OK?
		frame = dst;
	}


	/* store current frame as previous frame */
	prev_frame_keypoints = keypoints;
	prev_frame_descriptors = descriptors;
	frame_counter++;


	/* merge with cancas */
	int x, y;
	CvScalar s;

	for(x=0; x < frame->width; x++)
    {
        for(y=0;y < frame->height; y++)
        {
			s = cvGet2D(frame, y, x);
			if (s.val[0] != 0.0 || s.val[1] != 0.0 || s.val[2] != 0.0)
				cvSet2D(canvas, y + tmp_yoffset, x + tmp_xoffset, s);
        }
    }

	imshow("Image:", canvas);
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