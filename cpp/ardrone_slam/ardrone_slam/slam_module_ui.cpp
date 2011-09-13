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
}


void slam_module_ui::update()
{
	if (!initialized)
		init();


	if (clock() - prev_update > 0.5f * CLOCKS_PER_SEC)
	{
		int roi[4];

		if (controller->elevation_map.is_updated(roi, true))
			terrain->update_elevation_map(roi);

		if (controller->visual_map.is_updated(roi, true))
			terrain->update_texture(roi);

		controller->get_world_position(pos);
		controller->elevation_map.worldpos_to_cell(pos);

		prev_update = clock();

		terrain->render();

		return;
	}

	if (terrain->requires_render())
		terrain->render();
}


void slam_module_ui::display_canvas()
{
	Mat subCanvas(controller->visual_map.canvas, Rect(700, 1200, 2300, 2800));
	Mat resized(800, 800, CV_8UC4);
	resize(subCanvas, resized, Size(800, 800));

	cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
	imshow("Image:", resized);
	cvWaitKey(4);

	Sleep(99999);
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
		controller->visual_map.get_array(),
		pos
	);


	HWND consoleWindow = GetConsoleWindow();
	//SetFocus(consoleWindow);
	BringWindowToTop(consoleWindow);

	/* OpenCV window */
	//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);
}