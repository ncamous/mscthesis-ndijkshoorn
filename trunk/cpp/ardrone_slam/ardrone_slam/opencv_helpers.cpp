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
            } 
        } 
        printf("\n"); 
    } 
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

}