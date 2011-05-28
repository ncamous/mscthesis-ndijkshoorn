#pragma once

#include <windows.h>
#include <queue>

#include "opencv2/core/types_c.h"
#include "opencv2/features2d/features2d.hpp"

class slam_matlab;

enum slam_queue_type { CONTROL, MEASUREMENT, FRAME };

struct bot_ardrone_frame;
struct bot_ardrone_measurement;

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
	void process_frame(IplImage *i);
	void process_measurement(bot_ardrone_measurement *m);
	int find_features(IplImage *img, vector<cv::KeyPoint> &v);
	void calculate_frame_mask(int width, int height);

	// openCV helpers
	void frame_add_noise(IplImage *img);
	void PrintMat(CvMat *A);
	void dumpMatrix(const cv::Mat &mat);
	double MatMax(const cv::Mat &mat);
	double MatMin(const cv::Mat &mat);
	int MatNegCount(const cv::Mat &mat);
	double ColMin(const cv::Mat &mat, int col);
	double ColMax(const cv::Mat &mat, int col);

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
	int slam_queue_frames;

	int frame_counter;
	int feature_counter;
	double feature_distance;

	cv::Mat prev_frame_h;

	// sonar elevation map
	int last_loc[2];
	slam_matlab *matlab;
	int elevation;
	int initial_height;
	float canvas_scale; // px -> mm

	int dropped_frame_counter;

	cv::Mat obstacle_map;
	cv::Mat frame_mask;
};

