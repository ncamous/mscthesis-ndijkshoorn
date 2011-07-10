#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

struct bot_ardrone_frame;

class slam;

using namespace cv;

class slam_module_frame
{
public:
	slam_module_frame(slam *controller);
	~slam_module_frame(void);
	void process(bot_ardrone_frame *f);
	void process(IplImage *i);
	void compute_motion(Mat& cam_or, Mat& cam_pos, vector<DMatch>& matches, vector<char>& mask);
	int find_features(IplImage *img, vector<KeyPoint> &v);
	int find_robust_matches(vector<Point2f>& p1, vector<Point2f>& p2, vector<DMatch>& matches, vector<char>& mask, int max);

	void calculate_frame_mask(int width, int height);
	void add_noise(IplImage *img);
	void imagepoints_to_world3d(vector<Point2f>& src, vector<Point3f>& dst);
	void get_state(Mat& pos, Mat& or);
	void objectpos_to_localcam(Mat& pos, Mat& or, Mat& rot, bool state_provided=false);
	void objectpos_to_worldpos(Mat& pos, Mat& or);


private:
	slam *controller;

	IplImage *frame;
	IplImage *gray;

	// current frame data
	vector<Point2f> current_frame_ip;

	// previous frame data
	Mat prev_frame_descriptors;
	vector<Point2f> prev_frame_ip;
	vector<Point3f> prev_frame_wc;

	FeatureDetector *fd;
	DescriptorExtractor *de;
	BruteForceMatcher<L2<float>> dm;

	int frame_counter;

	Mat camera_matrix;
	Mat camera_matrix_inv;

	Mat world_plane;
	Mat world_plane_normal;

	Mat obstacle_map;
	Mat frame_mask;

	/* KF */
	KalmanFilter *KF;
	Mat *state;

	Mat measurement;
	Mat measurementMatrix;
	Mat measurementNoiseCov;
};

