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
	int find_robust_matches(vector<Point2f>& p1, vector<Point2f>& p2, vector<DMatch>& matches, vector<short>& mask, int max);
	int find_object_position(Mat& cam_pos, Mat& cam_or, vector<DMatch>& matches, vector<short>& mask);
	int find_features(IplImage *img, vector<KeyPoint> &v);

	void imagepoints_to_world3d(vector<Point2f>& src, vector<Point3f>& dst, bool swap_xy = false);
	
	void get_state(Mat& pos, Mat& or);
	void get_localcam(Mat& pos, Mat& or);
	void object_to_worldpos(Mat& obj_pos, Mat& obj_or, Mat& pos, Mat& or); // in: double!
	void get_objectpos(Mat& pos, Mat& or); // out: double!

	// coordinate system helpers
	void object_to_localcam(Mat& pos, Mat& or);
	void localcam_to_object(Mat& pos, Mat& or);
	void localcam_to_world(Mat& pos, Mat& or);
	void world_to_localcam(Mat& pos, Mat& or);


	void calculate_frame_mask(int width, int height);
	void add_noise(IplImage *img);


private:
	slam *controller;

	Mat frame;
	Mat frame_gray;
	Mat frame_rgba;

	// image corners
	vector<Point2f> image_corners;

	// current frame data
	vector<Point2f> current_frame_ip;

	// previous frame data
	Mat prev_frame_descriptors;
	vector<Point2f> prev_frame_ip;
	vector<Point3f> prev_frame_wc;

	//SurfFeatureDetector *fd;
	DescriptorExtractor *de;
	BruteForceMatcher<L2<float>> dm;

	int frame_counter;

	Mat camera_matrix;
	Mat camera_matrix_inv;

	Mat world_plane;
	Mat world_plane_normal;

	//Mat frame_mask;

	/* KF */
	KalmanFilter *KF;
	Mat *state;

	Mat measurement;
	Mat measurementMatrix;
	Mat measurementNoiseCov;
};

