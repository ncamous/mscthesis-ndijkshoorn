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
	void find_features(IplImage *img, vector<cv::KeyPoint> &v);

	bool CV_ready;
	IplImage *frame;
	IplImage *gray;
	cv::FeatureDetector *fd;
};

