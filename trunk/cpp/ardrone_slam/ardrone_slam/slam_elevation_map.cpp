#include "global.h"
#include "slam_elevation_map.h"


slam_elevation_map::slam_elevation_map(void):
	map(2 * SLAM_ELEVATION_MAP_DEFAULT_SIZE, 2 * SLAM_ELEVATION_MAP_DEFAULT_SIZE, CV_16S)
{
	map = 0;
	map_updated = false;
	memset(roi, -1, 4 * sizeof(int));

	w = 2 * SLAM_ELEVATION_MAP_DEFAULT_SIZE;
	h = 2 * SLAM_ELEVATION_MAP_DEFAULT_SIZE;
}

slam_elevation_map::~slam_elevation_map(void)
{

}

void slam_elevation_map::update(int x, int y, short h)
{
	if (map.at<short>(x, y) == h)
		return;

	map.at<short>(x, y) = h;
	update_roi(x, y);
	map_updated = true;
}

short* slam_elevation_map::get_array()
{
	return (short*) map.data;
}

bool slam_elevation_map::is_updated(int* roi, bool reset_roi)
{
	if (!map_updated)
		return false;

	memcpy_s(roi, 4 * sizeof(int), this->roi, 4 * sizeof(int));

	if (reset_roi)
	{
		memset(this->roi, -1, 4 * sizeof(int));
		map_updated = false;
	}

	return true;
}

void slam_elevation_map::update_roi(int x, int y)
{
	if (roi[0] == -1 || x < roi[0])
		roi[0] = x;
	
	if (roi[1] == -1 || x > roi[1])
		roi[1] = x;

	if (roi[2] == -1 || y < roi[2])
		roi[2] = y;

	if (roi[3] == -1 || y > roi[3])
		roi[3] = y;
}