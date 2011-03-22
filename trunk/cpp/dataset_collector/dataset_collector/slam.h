#pragma once

#include "opencv2/core/types_c.h"

struct bot_ardrone_frame;


class slam
{
public:
	slam(void);
	~slam(void);
	void process_frame(bot_ardrone_frame *f);
	void find_features(IplImage *img);

	//static slam* instance();
	//static slam* myinstance;

	//IplImage *frame_img;
};

