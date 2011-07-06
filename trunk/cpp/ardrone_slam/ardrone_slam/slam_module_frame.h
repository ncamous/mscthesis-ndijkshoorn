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
	int find_features(IplImage *img, std::vector<cv::KeyPoint> &v);
	void calculate_frame_mask(int width, int height);
	void add_noise(IplImage *img);
	void imagepoints_to_world3d(std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dst);
	void get_current_camera(cv::Mat& pos, cv::Mat& orientation);


private:
	slam *controller;

	IplImage *frame;
	IplImage *gray;

	std::vector<cv::KeyPoint> prev_frame_keypoints;
	cv::Mat prev_frame_descriptors;
	std::vector<cv::Point3f> prev_frame_wc;

	cv::FeatureDetector *fd;
	cv::DescriptorExtractor *de;
	cv::BruteForceMatcher<cv::L2<float>> dm;

	int frame_counter;
	int feature_counter;
	int dropped_frame_counter;
	double feature_distance;

	cv::Mat camera_matrix;
	cv::Mat camera_matrix_inv;

	cv::Mat world_plane;
	cv::Mat world_plane_normal;

	cv::Mat prev_frame_h;

	cv::Mat obstacle_map;
	cv::Mat frame_mask;

	cv::KalmanFilter *KF;
	cv::Mat *state;
};

