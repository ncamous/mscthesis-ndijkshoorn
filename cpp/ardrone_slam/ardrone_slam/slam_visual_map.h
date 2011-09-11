#pragma once

#include <cv.hpp>


struct CornerHist
{
	cv::Mat hist;
	cv::Point2f cc; // canvas coordinate
	cv::Point2f wc; // local world coordinate
};


class slam_visual_map
{
public:
	slam_visual_map();
	~slam_visual_map();
	byte* get_array();
	void update(cv::Mat& frame, std::vector<cv::Point2f>& lc, std::vector<cv::Point3f>& wc);
	void frame_to_canvas(cv::Mat& frame, cv::Mat& frameT, std::vector<cv::Point2f>& lc, std::vector<cv::Point3f>& wc);
	bool is_updated(int* dst, bool reset_roi = false);
	void find_corners(cv::Mat& img, std::vector<CornerHist>& list, bool unique = false);
	bool corner_at_wc(cv::Point2f wc);

	cv::Mat canvas;

	/* corners */
	std::vector<CornerHist> corners;
	int channels[2];
	int h_bins;
	int s_bins;
	int histSize[2];

	// hue varies from 0 to 256, saturation from 0 to 180
	float h_ranges[2];
	float s_ranges[2];
	const float* ranges[2];

	float resolution;
	float resolution_inv;

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

	cv::TermCriteria termcrit;

	void worldpos_to_cell(cv::Point3f& src, cv::Point2f& dst);
	void cell_to_worldpos(cv::Point2f& src, cv::Point2f& dst);
	void update_roi(cv::Point2f& p, int *roi);
	void update_roi(int *src, int *dst);
};