#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/features2d/features2d.hpp"

struct bot_ardrone_frame;


class slam
{
public:
	slam(void);
	~slam(void);
	void init_openCV();
	void process_frame(bot_ardrone_frame *f);
	void find_features(IplImage *img, vector<cv::KeyPoint> &v);

	bool openCV_init;
	IplImage *frame;
	IplImage *gray;
	cv::FeatureDetector *fd;
};

