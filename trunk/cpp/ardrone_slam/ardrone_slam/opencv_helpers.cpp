#include "opencv_helpers.h"
#include <cv.hpp>


namespace cv {

	void PrintMat(CvMat *A)
	{
		int i, j;
		for (i = 0; i < A->rows; i++)
		{
		printf("\n"); 
		switch (CV_MAT_DEPTH(A->type))
		{
		case CV_32F:
		case CV_64F:
		for (j = 0; j < A->cols; j++)
		printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
		break;
		case CV_8U:
		case CV_16U:
		for(j = 0; j < A->cols; j++)
		printf ("%6d",(int)cvGetReal2D(A, i, j));
		break;
		default:
		break;
		}
		}
		printf("\n");
	}

	void dumpMatrix(const Mat &mat) { 
		const int t = mat.type(); 
		for (int i = 0; i < mat.rows; i++) { 
			for (int j = 0; j < mat.cols; j++) { 
				switch (t) { 
				case CV_32F: 
					printf("%6.4f, ", mat.at<float> (i, j)); 
					break; 
				case CV_64F: 
					printf("%6.4f, ", mat.at<double> (i, j)); 
					break; 
				case CV_8UC1:
					printf("%i, ", (int) mat.at<char> (i, j)); 
					break; 
				} 
			} 
			printf("\n"); 
		} 
		printf("\n");
	} 

	double MatMax(const Mat &mat)
	{
		double max = 0.0;
		double *vals = (double*) mat.data;
		int elements = mat.rows * mat.cols;

		for (int i = 0; i < elements; i++)
		{
			if (vals[i] > max)
				max = vals[i];
		}

		return max;
	}

	double MatMin(const Mat &mat)
	{
		double min = 0.0;
		double *vals = (double*) mat.data;
		int elements = mat.rows * mat.cols;

		for (int i = 0; i < elements; i++)
		{
			if (vals[i] < min)
				min = vals[i];
		}

		return min;
	}

	int MatNegCount(const Mat &mat)
	{
		int neg_count = 0;
		double *vals = (double*) mat.data;
		int elements = mat.rows * mat.cols;

		for (int i = 0; i < elements; i++)
		{
			if (vals[i] < 0.0)
				neg_count++;
		}

		return neg_count;
	}

	double ColMin(const Mat &mat, int col)
	{
		double min = FLT_MAX;

		for (int i = 0; i < mat.rows; i++)
		{
			if (mat.at<double> (i, col) < min)
				min = mat.at<double> (i, col);
		}

		return min;
	}

	double ColMax(const Mat &mat, int col)
	{
		double max = FLT_MIN;

		for (int i = 0; i < mat.rows; i++)
		{
			if (mat.at<double> (i, col) > max)
				max = mat.at<double> (i, col);
		}

		return max;
	}

	void RotationMatrix3D(const Mat& src_m, Mat& dst_m, bool yawlast)
	{
		//float* dst = (float*) dst_m.data;
		float* src = (float*) src_m.data; // angles in rad

		float CosRx = cos(src[0]);
		float CosRy = cos(src[1]);
		float CosRz = cos(src[2]);
		float SinRx = sin(src[0]);
		float SinRy = sin(src[1]);
		float SinRz = sin(src[2]);

		// http://www.codecogs.com/pages/forums/pagegen.php?id=455
		// http://www.intechopen.com/source/pdfs/11307/InTech-Visual_slam_and_moving_object_detection_for_a_small_size_humanoid_robot.pdf
		// http://46dogs.blogspot.com/2011/04/right-handed-rotation-matrix-for.html

		/* Note: this matrix effectively yaws first, then pitches, then rolls.  For the opposite order (and the one I ended up using), look below.*/
		if (!yawlast)
		{
			dst_m.at<float>(0, 0) = CosRy * CosRz;
			dst_m.at<float>(0, 1) = -SinRz * CosRy;
			dst_m.at<float>(0, 2) = SinRy;

			dst_m.at<float>(1, 0) = CosRz * SinRy * SinRx + SinRz * CosRx;
			dst_m.at<float>(1, 1) = -SinRz * SinRy * SinRx + CosRz * CosRx;
			dst_m.at<float>(1, 2) = -CosRy * SinRx;

			dst_m.at<float>(2, 0) = -CosRz * SinRy * CosRx + SinRz * SinRx;
			dst_m.at<float>(2, 1) = SinRz * SinRy * CosRx + CosRz * SinRx;
			dst_m.at<float>(2, 2) = CosRy * CosRx;
		}

		/* Note: this matrix effectively rolls first, then pitches, then yaws.  This turned out to be the one I used for my system.*/
		else
		{
			dst_m.at<float>(0, 0) = CosRy * CosRz;
			dst_m.at<float>(0, 1) = -SinRz * CosRx + CosRz * SinRy * SinRx;
			dst_m.at<float>(0, 2) = SinRx * SinRz + CosRz * SinRy * CosRx;

			dst_m.at<float>(1, 0) = CosRy * SinRz;
			dst_m.at<float>(1, 1) = CosRz * CosRx + SinRy * SinRz * SinRx;
			dst_m.at<float>(1, 2) = -SinRx * CosRz + SinRy * SinRz * CosRx;

			dst_m.at<float>(2, 0) = -SinRy;
			dst_m.at<float>(2, 1) = CosRy * SinRx;
			dst_m.at<float>(2, 2) = CosRy * CosRx;
		}
	}


	void TransformationMatrix(const Mat& pos, const Mat& or, Mat& T)
	{
		RotationMatrix3D(or, T);

		T.at<float>(0, 3) = pos.at<float>(0);
		T.at<float>(1, 3) = pos.at<float>(1);
		T.at<float>(2, 3) = pos.at<float>(2);
	}


	void CalcLinePlaneIntersection(const Mat& Plane, const Mat& PlaneNormal, const Mat& Line, const Mat& LineNormal, Mat& intersection)
	{
		double d = 0;
		double t = (d - PlaneNormal.dot(Line)) / PlaneNormal.dot(LineNormal);
		addWeighted(Line, 1.0, LineNormal, t, 0.0, intersection);
	}


	void CalcLinePositionAtDistance(const Mat& Line, const Mat& LineNormal, double d, Mat& intersection)
	{
		addWeighted(Line, 1.0, LineNormal, d, 0.0, intersection);
	}


	void MatFloatToDouble(const Mat &in, Mat &out)
	{
		for (int i = 0; i < in.rows; i++)
		{
			for (int j = 0; j < in.cols; j++)
			{
				out.at<double>(i, j) = (double) in.at<float>(i, j);
			}
		}
	}


	void MatDoubleToFloat(const Mat &in, Mat &out)
	{
		for (int i = 0; i < in.rows; i++)
		{
			for (int j = 0; j < in.cols; j++)
			{
				out.at<float>(i, j) = (float) in.at<double>(i, j);
			}
		}
	}

	void MatSetDiag(Mat &mtx, float *diag)
	{
		int rows = mtx.rows;

		for (int i = 0; i < rows; i++)
			mtx.at<float>(i, i) = diag[i];
	}

}