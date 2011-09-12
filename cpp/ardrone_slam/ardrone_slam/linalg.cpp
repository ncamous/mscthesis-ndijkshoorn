/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#include "linalg.h"


// functions that return scalars

double norm(double vect[2]) {
	return sqrt(dot(vect, vect));
}

double dot(double vect1[2], double vect2[2]) {
	return vect1[0] * vect2[0] + vect1[1] * vect2[1];
}

double dist(double vect1[2], double vect2[2]) {
	/*double vect3[2];
	diff(vect1, vect2, vect3);
	return norm(vect3);
	*/
	return sqrt( (vect1[0]-vect2[0]) * (vect1[0]-vect2[0]) + (vect1[1]-vect2[1]) * (vect1[1]-vect2[1]) );
}



// functions that return vectors (using pointer in last argument)

void unit(double vect_in[2], double vect_out[2]) {
	double length = norm(vect_in);
	if (length == 0) {
		vect_out[0] = 0;
		vect_out[1] = 0;
	} else {
		vect_out[0] = vect_in[0] / length;
		vect_out[1] = vect_in[1] / length;
	}
}

void add(double vect1[2], double vect2[2], double vect_out[2]) {
	vect_out[0] = vect1[0] + vect2[0];
	vect_out[1] = vect1[1] + vect2[1];
}

void diff(double vect1[2], double vect2[2], double vect_out[2]) {
	vect_out[0] = vect1[0] - vect2[0];
	vect_out[1] = vect1[1] - vect2[1];
}

// angle in rad
void rot(double vect_in[2], double angle, double vect_out[2]) {
	double x = vect_in[0];
	double y = vect_in[1];
	vect_out[0] = cos(angle) * x - sin(angle) * y;
	vect_out[1] = sin(angle) * x + cos(angle) * y;
}

void cp(double vect_in[2], double vect_out[2]) {
	vect_out[0] = vect_in[0];
	vect_out[1] = vect_in[1];
}



// functions that return vectors (using pointer in last argument)
// true: when traveling from a via b to c, you make a curve to the right
bool sign(double a[2], double b[2], double c[2]) {
	return (c[0]*a[1] - c[0]*b[1] + c[1]*b[0] - c[1]*a[0] + b[1]*a[0] - b[0]*a[1]) > 0;
}