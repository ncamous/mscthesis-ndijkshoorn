#include "slam.h"
#include "bot_ardrone.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

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

		printf("%i, %i\n", w, h);

		IplImage *frame_img = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
	//}

	frame_img->imageData = &f->data[4];
	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
	cvShowImage("Image:", frame_img);

	// Wait for the user to press a key in the GUI window.
	cvWaitKey(2);
}