#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/features2d/features2d.hpp"

struct bot_ardrone_frame;


class slam
{
public:
	slam(void);
	~slam(void);
	void init_CV();
	void process_frame(bot_ardrone_frame *f);
	void simpleMatching(cv::DescriptorMatcher *descriptorMatcher, const cv::Mat& descriptors1, const cv::Mat& descriptors2, vector<cv::DMatch>& matches12);
	void find_features(IplImage *img, vector<cv::KeyPoint> &v);

	bool CV_ready;
	IplImage *canvas;
	IplImage *frame;
	IplImage *gray;
	vector<cv::KeyPoint> prev_frame_keypoints;
	cv::Mat *prev_frame_descriptors;
	cv::FeatureDetector *fd;
	cv::DescriptorExtractor *de;
	cv::DescriptorMatcher *dm;

	int tmp_xoffset;
	int tmp_yoffset;
};

