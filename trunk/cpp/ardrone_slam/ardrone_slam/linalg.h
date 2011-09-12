/* Copyright 2011  Martijn van der Veen
 * GPL version 2 (or later)
 * For the full notice, see COPYRIGHT.txt
 */

#pragma once
#include <math.h>
// Linear algebra helper functions

// functions that return scalars
double norm(double vect[2]);
double dot(double vect1[2], double vect2[2]);
double dist(double vect1[2], double vect2[2]);

// functions that return vectors (using pointer in last argument)
void unit(double vect_in[2], double vect_out[2]);
void add(double vect1[2], double vect2[2], double vect_out[2]);
void diff(double vect1[2], double vect2[2], double vect_out[2]);
void rot(double vect_in[2], double angle, double vect_out[2]);
void cp(double vect_in[2], double vect_out[2]);


// special helper functions
bool sign(double vect1[2], double vect2[2], double vect3[2]);
