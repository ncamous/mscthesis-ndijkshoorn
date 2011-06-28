#pragma once

#include "opencv2/core/core.hpp"

namespace cv {
	void PrintMat(CvMat *A);
	void dumpMatrix(const Mat &mat);
	double MatMax(const Mat &mat);
	double MatMin(const Mat &mat);
	int MatNegCount(const Mat &mat);
	double ColMin(const Mat &mat, int col);
	double ColMax(const Mat &mat, int col);
	void RotationMatrix3D(const Mat& src, Mat& dst);
}