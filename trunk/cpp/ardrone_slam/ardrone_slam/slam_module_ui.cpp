#include "global.h"
#include "slam_module_ui.h"
#include "slam.h"

#include <opencv2/highgui/highgui.hpp>

using namespace cv;


slam_module_ui::slam_module_ui(slam *controller)
{
	this->controller = controller;

	CV_ready = false;
}


slam_module_ui::~slam_module_ui(void)
{
	cvDestroyWindow("Image:");
}


void slam_module_ui::update()
{
	if (!CV_ready)
		init_CV();

	Mat img(controller->canvas);
	//imshow("Image:", controller->canvas);
	cvWaitKey(4);
}


void slam_module_ui::init_CV()
{
	CV_ready = true;

	if (SLAM_USE_OBSTACLE_MASK)
		cvNamedWindow("Mask:", CV_WINDOW_AUTOSIZE);

	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);


	/* obstacle map */
	/*
	if (This->CV_ready && SLAM_USE_OBSTACLE_MASK)
	{
		cvNamedWindow("Obstacles:", CV_WINDOW_AUTOSIZE);
		IplImage *obstacle_img = cvCreateImage(cvSize(800, 800), 8, 1);
		obstacle_img->imageData = (char*)This->obstacle_map.data;
		imshow("Obstacles:", obstacle_img);
		cvWaitKey(4);
	}

	//printf("Display Matlab\n");
	//This->matlab->display();
	//Sleep(99999);
	*/
}