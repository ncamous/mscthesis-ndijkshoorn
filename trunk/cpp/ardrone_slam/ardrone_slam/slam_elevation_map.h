#pragma once

#include <cv.hpp>


class slam_elevation_map
{
public:
	slam_elevation_map();
	~slam_elevation_map();
	void update(int x, int y, short h);
	short* get_array();
	bool is_updated(int* dst, bool reset_roi = false);

	unsigned int w;
	unsigned int h;

	bool map_updated;
	int roi[4];

private:
	cv::Mat map;

	void update_roi(int x, int y);
};