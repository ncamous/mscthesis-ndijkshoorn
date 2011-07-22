#include "global.h"
#include "slam_module_ui.h"
#include "slam.h"

#include "terrain3d.h"

#include <opencv2/highgui/highgui.hpp>

using namespace cv;


slam_module_ui::slam_module_ui(slam *controller)
{
	this->controller = controller;

	initialized = false;
}


slam_module_ui::~slam_module_ui(void)
{
	cvDestroyWindow("Image:");
}


void slam_module_ui::update()
{
	if (!initialized)
		init();

	int roi[4];

	if (controller->elevation_map.is_updated(roi, true))
	{
		printf("ROI: %i, %i, %i, %i\n", roi[0], roi[1], roi[2], roi[3]);
		terrain->update_elevation_map(roi);
	}

	terrain->render();

	//Mat img(controller->canvas);
	//imshow("Image:", controller->canvas);
	//cvWaitKey(4);
}


void slam_module_ui::init()
{
	initialized = true;


	/* 3D Terrain */
	terrain = new terrain3d(
		controller->elevation_map.get_array(),
		controller->elevation_map.w,
		controller->elevation_map.h
	);

	/* OpenCV window */
	//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);


	/* Matlab */
	/*
	if (!(matlab = engOpen("\0"))) {
		printf("\nCan't start MATLAB engine\n");
		return;
	}

	//engEvalString(matlab, "[X,Y] = meshgrid(-100:99,-100:99);");
	engEvalString(matlab, "[X,Y] = meshgrid(-10:9,-10:9);");
	*/
}