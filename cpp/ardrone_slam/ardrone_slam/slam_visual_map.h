#pragma once

#include <cv.hpp>


class slam_visual_map
{
public:
	slam_visual_map();
	~slam_visual_map();
	byte* get_array();
	void update(cv::Mat& frame, std::vector<cv::Point2f>& lc, std::vector<cv::Point3f>& wc);
	bool is_updated(int* dst, bool reset_roi = false);

	cv::Mat canvas;

	float resolution;

	unsigned int w;
	unsigned int h;

	unsigned int origin_x;
	unsigned int origin_y;

	bool map_updated;
	int frame_roi[4];
	int sync_roi[4];

private:
	cv::Size canvasSize;
	cv::Mat undoTranslate;

	void worldpos_to_cell(cv::Point3f& src, cv::Point2f& dst);
	void update_roi(cv::Point2f& p, int *roi);
	void update_roi(int *src, int *dst);
};