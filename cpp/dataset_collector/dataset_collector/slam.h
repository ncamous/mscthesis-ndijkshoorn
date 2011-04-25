#pragma once

#include <windows.h>
#include <queue>

#include "opencv2/core/types_c.h"
#include "opencv2/features2d/features2d.hpp"


enum slam_queue_type { CONTROL, MEASUREMENT, FRAME };

struct bot_ardrone_frame;

struct slam_queue_item {
	slam_queue_type type;
	void *object;
};

typedef queue<slam_queue_item> SLAMQUEUE;

static DWORD WINAPI process_thread(void* Param);

class slam
{
public:
	slam();
	~slam(void);
	void slam::run();
	void init_CV();
	void process_frame(bot_ardrone_frame *f);
	void process_frame_test(IplImage *f);
	void find_features(IplImage *img, vector<cv::KeyPoint> &v);

	void PrintMat(CvMat *A);
	void dumpMatrix(const cv::Mat &mat);

	HANDLE h;
	HANDLE slam_queue_pushed;
	HANDLE slam_queue_empty;

	bool CV_ready;
	IplImage *canvas;
	IplImage *frame;
	IplImage *gray;
	vector<cv::KeyPoint> prev_frame_keypoints;
	cv::Mat prev_frame_descriptors;
	cv::FeatureDetector *fd;
	cv::DescriptorExtractor *de;
	cv::BruteForceMatcher<cv::L2<float>> dm;

	SLAMQUEUE slam_queue;

	int frame_counter;

	cv::Mat prev_frame_h;
};

