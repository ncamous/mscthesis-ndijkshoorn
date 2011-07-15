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

	//Mat img(controller->canvas);
	//imshow("Image:", controller->canvas);
	//cvWaitKey(4);

	display_elevation_map();
}


void slam_module_ui::display_elevation_map()
{
	mxArray *elevation_map_tuples;
	double *elevation_map_dst;
	short *elevation_map_src = (short*) controller->elevation_map.data;

	int m = controller->elevation_map.rows;
	int n = controller->elevation_map.cols;

	// http://www.mathworks.com/help/techdoc/apiref/mxcreatenumericmatrix.html
	elevation_map_tuples = mxCreateNumericMatrix(m, n, mxDOUBLE_CLASS/*mxINT8_CLASS*/, mxREAL);
	elevation_map_dst = (double*) mxGetPr(elevation_map_tuples);

	//memcpy_s(elevation_map_dst, m * n * sizeof(char), elevation_map_src, m * n * sizeof(char));

	for (int x = 0; x < m; x++)
	{
		for (int y = 0; y < n; y++)
		{
			elevation_map_dst[y * n + x] = (double) elevation_map_src[y * n + x];
		}
	}


	engPutVariable(matlab, "T", elevation_map_tuples);
	engEvalString(matlab, "Z = T(90:109, 90:109);");
	engEvalString(matlab, "figure(1);");
	//engEvalString(matlab, "mesh(X,Y,Z,'EdgeColor','none');");
	engEvalString(matlab, "mesh(X,Y,Z);");
	engEvalString(matlab, "axis([-10 9 -10 9 -10 250]);");
	//engEvalString(matlab, "colormap('winter');");
	//engEvalString(matlab, "contour(X,Y,Z);");

	mxDestroyArray(elevation_map_tuples);
}


void slam_module_ui::init_CV()
{
	CV_ready = true;

	//cvNamedWindow("Image:", CV_WINDOW_AUTOSIZE);


	/* Matlab */
	if (!(matlab = engOpen("\0"))) {
		printf("\nCan't start MATLAB engine\n");
		return;
	}

	// once
	//engEvalString(matlab, "[X,Y] = meshgrid(-100:99,-100:99);");
	engEvalString(matlab, "[X,Y] = meshgrid(-10:9,-10:9);");
}