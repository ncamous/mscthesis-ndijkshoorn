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
	void RotationMatrix3D(const Mat& src, Mat& dst, bool yawlast=true);
	void TransformationMatrix(const Mat& pos, const Mat& or, Mat& T);
	void CalcLinePlaneIntersection(const Mat& Plane, const Mat& PlaneNormal, const Mat& Line, const Mat& LineNormal, Mat& intersection);
	void CalcLinePositionAtDistance(const Mat& Line, const Mat& LineNormal, double d, Mat& intersection);
	void MatFloatToDouble(const Mat &in, Mat &out);
	void MatDoubleToFloat(const Mat &in, Mat &out);
	void MatSetDiag(Mat &mtx, float *diag);
}