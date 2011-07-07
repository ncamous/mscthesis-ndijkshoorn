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
	int find_robust_matches(std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2, std::vector<cv::DMatch>& matches, cv::vector<char>& matchesMask);
	void calculate_frame_mask(int width, int height);
	void add_noise(IplImage *img);
	void imagepoints_to_world3d(std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dst);
	void get_current_camera(cv::Mat& pos, cv::Mat& orientation);


private:
	slam *controller;

	IplImage *frame;
	IplImage *gray;

	cv::Mat prev_frame_descriptors;
	std::vector<cv::Point2f> prev_frame_ip;
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

	cv::Mat obstacle_map;
	cv::Mat frame_mask;

	/* KF */
	cv::KalmanFilter *KF;
	cv::Mat *state;

	cv::Mat measurement;
	cv::Mat measurementMatrix;
	cv::Mat measurementNoiseCov;
};

