#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

struct bot_ardrone_frame;

class slam;

class slam_module_frame
{
public:
	slam_module_frame(slam *controller);
	~slam_module_frame(void);
	void process(bot_ardrone_frame *f);
	void process(IplImage *i);
	int find_features(IplImage *img, vector<cv::KeyPoint> &v);
	void calculate_frame_mask(int width, int height);
	void add_noise(IplImage *img);


private:
	slam *controller;

	IplImage *frame;
	IplImage *gray;
	vector<cv::KeyPoint> prev_frame_keypoints;
	cv::Mat prev_frame_descriptors;

	cv::FeatureDetector *fd;
	cv::DescriptorExtractor *de;
	cv::BruteForceMatcher<cv::L2<float>> dm;

	int frame_counter;
	int feature_counter;
	int dropped_frame_counter;
	double feature_distance;

	cv::Mat prev_frame_h;

	cv::Mat obstacle_map;
	cv::Mat frame_mask;

	int last_loc[2];

	cv::KalmanFilter *KF;
	cv::Mat *state;
	float estimated_pos[3];
};

