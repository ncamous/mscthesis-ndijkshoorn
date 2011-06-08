#pragma once

#ifdef _CHAR16T
#define CHAR16_T
#endif

#include "engine.h"


class slam_matlab
{
public:
	slam_matlab(void);
	~slam_matlab(void);
	void slam_matlab::add_elevation_map_tuple(int *t);
	void display();

	mxArray *elevation_map_tuples;
	int *elevation_map_ptr;

private:
	int elevation_map_tuple;
};

