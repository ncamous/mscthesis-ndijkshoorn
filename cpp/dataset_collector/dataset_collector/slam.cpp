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

	de = new SurfDescriptorExtractor();

	dm = new BruteForceMatcher<L2<float> >();

	canvas = cvCreateImage( cvSize(800,800), 8, 3 );

	tmp_xoffset = 0;
	tmp_yoffset = 0;

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
	//cvCvtColor(frame, gray, CV_RGB2GRAY);

	//drawKeypoints(gray, keypoints, out);


	/* match with previous frame */
	Mat descriptors;
    de->compute(gray, keypoints, descriptors);

	if (prev_frame_descriptors == NULL)
	{
		prev_frame_keypoints = keypoints;
		prev_frame_descriptors = &descriptors;
		return;
	}

	Mat H12;
	vector<DMatch> matches;
	double ransacReprojThreshold = 3.0;
	simpleMatching(dm, descriptors, *prev_frame_descriptors, matches);


	/* calculate transformation (RANSAC) */
	double tt = (double)cvGetTickCount();

	vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        queryIdxs[i] = matches[i].queryIdx;
        trainIdxs[i] = matches[i].trainIdx;
    }

    if( ransacReprojThreshold >= 0 )
    {
        printf("Computing homography (RANSAC)...\n");

        vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);

        vector<Point2f> points2;

		KeyPoint::convert(prev_frame_keypoints, points2);

        H12 = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );
    }

/*
    Mat drawImg;
    if( !H12.empty() ) // filter outliers
    {
        vector<char> matchesMask( matches.size(), 0 );

        vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);
        vector<Point2f> points2;
		KeyPoint::convert(prev_frame_keypoints, points2, trainIdxs);

        Mat points1t;
		perspectiveTransform(Mat(points1), points1t, H12);

        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
            if( norm(points2[i1] - points1t.at<Point2f>((int)i1,0)) < 4 ) // inlier
                matchesMask[i1] = 1;
        }
	}
*/

	tt = (double)cvGetTickCount() - tt;

	printf( "RANASC transformation time = %gms\n", tt/(cvGetTickFrequency()*1000.));


	/* store current frame as previous frame */
	prev_frame_keypoints = keypoints;
	prev_frame_descriptors = &descriptors;

	/* 
	cvWarpPerspective( const CvArr* src, CvArr* dst, const CvMat* map_matrix,
                        int flags=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
                        CvScalar fillval=cvScalarAll(0) );
	*/
	IplImage out;
	cvWarpPerspective( frame, out, H12);

	int x, y;

	for(x=0; x < frame->width; x++)
    {
        for(y=0;y < frame->height; y++)
        {
            cvSet2D(canvas, y + tmp_yoffset, x + tmp_xoffset, cvGet2D(frame, y, x));
        }
    }

	//tmp_xoffset += 10;
	tmp_yoffset += 10;

	imshow("Image:", canvas);
	/**/

	cvWaitKey(2);
}


void slam::simpleMatching(DescriptorMatcher *descriptorMatcher, const Mat& descriptors1, const Mat& descriptors2, vector<DMatch>& matches12)
{
	double tt = (double)cvGetTickCount();

    vector<DMatch> matches;
    descriptorMatcher->match(descriptors1, descriptors2, matches12);

	tt = (double)cvGetTickCount() - tt;

	printf( "Matching time = %gms\n", tt/(cvGetTickFrequency()*1000.));
}


void slam::find_features(IplImage *img, vector<cv::KeyPoint> &v)
{
	double tt = (double)cvGetTickCount();

	fd->detect(img, v);

	tt = (double)cvGetTickCount() - tt;

	printf( "Extraction time = %gms\n", tt/(cvGetTickFrequency()*1000.));
}