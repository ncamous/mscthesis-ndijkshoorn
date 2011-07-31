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
	//cvDestroyWindow("Image:");
}


void slam_module_ui::update()
{
	if (!initialized)
		init();

	terrain->handle_input(); // mouse and keyboard

	if (clock() - prev_update > 2 * CLOCKS_PER_SEC)
	{
		int roi[4];

		if (controller->elevation_map.is_updated(roi, true))
			terrain->update_elevation_map(roi);

		if (controller->visual_map.is_updated(roi, true))
			terrain->update_texture(roi);

		prev_update = clock();
	}

	if (terrain->requires_render())
		terrain->render();

	/*
	Mat blup(controller->visual_map.canvas, Rect(1800, 1800, 600, 600));
	imshow("Image:", blup);
	cvWaitKey(4);
	*/
}


void slam_module_ui::init()
{
	initialized = true;

	prev_update = clock();


	/* 3D Terrain */
	terrain = new terrain3d(
		controller->elevation_map.get_array(),
		controller->elevation_map.w,
		controller->elevation_map.h,
		controller->visual_map.get_array()
	);


	/* OpenCV window */
	//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
}