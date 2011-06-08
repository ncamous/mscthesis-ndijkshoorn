#include "global.h"
#include "slam_matlab.h"

#include <memory.h>


slam_matlab::slam_matlab(void)
{
	elevation_map_tuple = 0;

	elevation_map_tuples = mxCreateNumericMatrix(3, 1000, mxINT32_CLASS, mxREAL);
	elevation_map_ptr = (int*) mxGetPr(elevation_map_tuples);
}


slam_matlab::~slam_matlab(void)
{
}


void slam_matlab::add_elevation_map_tuple(int *t)
{
	// matrix is full...
	if (elevation_map_tuple >= 1000)
		return;

	memcpy(&elevation_map_ptr[elevation_map_tuple * 3], &t[0], sizeof(int) * 3);

	elevation_map_tuple++;
}


void slam_matlab::display()
{
	Engine *ep;

	if (!(ep = engOpen("\0"))) {
		printf("\nCan't start MATLAB engine\n");
		return;
	}

	engPutVariable(ep, "T", elevation_map_tuples);

	/*
	 * Plot the result
	 */
	//engEvalString(ep, "X = [0, 1, 2, 3, 0, 1, 2, 3]");
	//engEvalString(ep, "Y = [0, 0, 0, 0, 1, 1, 1, 1]");
	//engEvalString(ep, "Z = [0, 0, 0, 0, 1, 0, 0, 0]");
	//engEvalString(ep, "points = [0,0,0 ; 300, 300, 50 ; 800, 800, 0];");
	//engEvalString(ep, "Z(300, 300) = 50;");
		//engEvalString(ep, "scatter3(T(1,:), T(2,:), T(3,:));");
	engEvalString(ep, "plot(T(3,:));");
	//engEvalString(ep, "surfc(X,Y,Z);");
	//engEvalString(ep, "plot3(1,1,1);");
	//engEvalString(ep, "colormap hsv;");

	mxDestroyArray(elevation_map_tuples);
	//engEvalString(ep, "close;");
}