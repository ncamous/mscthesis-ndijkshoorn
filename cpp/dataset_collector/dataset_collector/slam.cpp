#include "slam.h"
#include "bot_ardrone.h"

#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;
using namespace std;


/*
slam* slam::instance()
{
	return myinstance;
}
*/


slam::slam()
{
	//myinstance = (slam*) this;

	//frame_img = NULL;

	// Display the image.
	//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
}


slam::~slam()
{
	cvDestroyWindow("Image:");
}


void slam::process_frame(bot_ardrone_frame *f)
{
	//if (frame_img == NULL)
	//{
		unsigned short w, h;

		memcpy_s(&w, 2, &f->data[0], 2);
		memcpy_s(&h, 2, &f->data[2], 2);

		w = htons(w);
		h = htons(h);

		//printf("%i, %i\n", w, h);

		IplImage *frame_img = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
	//}

	frame_img->imageData = &f->data[4];

	find_features(frame_img);

	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
	cvShowImage("Image:", frame_img);

	// Wait for the user to press a key in the GUI window.
	cvWaitKey(2);
}


void slam::find_features(IplImage *img)
{
	FeatureDetector* fd;

	/*
	SurfFeatureDetector( double hessianThreshold=400., int octaves=3, int octaveLayers=4 );
	*/

	fd = new SurfFeatureDetector(2200., 2, 3);


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

	double tt = (double)cvGetTickCount();
	vector<KeyPoint> keypoints;

	fd->detect(img, keypoints);

	tt = (double)cvGetTickCount() - tt;
	//printf( "Extraction time = %gms\n", tt/(cvGetTickFrequency()*1000.));

	//printf("nr features: %i\n", keypoints.size());
}