#pragma once

#include "slam_visual_map.h"
#include "opencv2/core/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

#define SLAM_LOC_START controller->sensor_pause(f->time);
#define SLAM_LOC_END controller->sensor_resume(); Sleep(75);

struct bot_ardrone_frame;

class slam;

using namespace cv;

class slam_module_frame
{
public:
	slam_module_frame(slam *controller);
	~slam_module_frame(void);
	void set_camera();

	void process(bot_ardrone_frame *f);
	void process_visual_state();
	void process_visual_loc();
	void process_map();

	double find_robust_translation(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& inliers, cv::Mat& T, double maxInlierDist = 3.0);
	int find_robust_matches(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& mask, int max, cv::Mat& H, double maxInlierDist = 3.0);
	int find_object_position(Mat& cam_pos, Mat& cam_or, vector<DMatch>& matches, vector<short>& mask);

	void get_features(Mat& frame, vector<KeyPoint> &v);
	void get_descriptors(Mat& frame, vector<KeyPoint> &v, Mat& descriptors);
	void get_matches(Mat& q_descriptors, Mat& t_descriptors, vector<DMatch>& matches, bool use_unique = false);

	void store_prev_frame();
	void calculate_measurement();
	void save_cur_state();
	bool measurementSeemsOk();

	void imagepoints_to_local3d(vector<Point2f>& src, vector<Point3f>& dst);
	void imagepoints_to_local3d(vector<Point2f>& src, vector<Point2f>& dst);

	void get_state(Mat& pos, Mat& or);
	void get_localcam(Mat& pos, Mat& or);
	void object_to_worldpos(Mat& obj_pos, Mat& obj_or, Mat& pos, Mat& or); // in: double!
	void get_objectpos(Mat& pos, Mat& or); // out: double!

	// coordinate system helpers
	void object_to_localcam(Mat& pos, Mat& or);
	void localcam_to_object(Mat& pos, Mat& or);
	void localcam_to_world(Mat& pos, Mat& or);
	void world_to_localcam(Mat& pos, Mat& or);

	// testing/experiments
	void add_noise(IplImage *img);


private:
	slam *controller;

	bot_ardrone_frame *f;

	Mat frame;
	Mat frame_gray;
	//Mat frame_rgba;

	Mat obj_pos;
	Mat obj_or;
	Mat new_pos;
	Mat new_or;

	// image corners
	vector<Point2f> image_corners;

	// current frame data
	vector<KeyPoint> keypoints;
	vector<Point2f> imagepoints;
	Mat descriptors;

	// previous frame data
	bool prev_frame_exists;
	double prev_frame_time;
	Mat prev_frame_descriptors;
	vector<Point2f> prev_frame_ip;
	vector<Point3f> prev_frame_wc;

	//SurfFeatureDetector *fd;
	DescriptorExtractor *de;
	BruteForceMatcher<L2<float>> dm;

	Mat camera_matrix;
	Mat camera_matrix_inv;

	Mat world_plane;
	Mat world_plane_normal;

	Mat T;
	Mat originH;

	/* KF */
	KalmanFilter *KF;
	Mat *state;
	Mat prev_state;
	Mat cur_state;

	Mat measurement;
	Mat measurementMatrix;
	Mat measurementNoiseCov;

	double difftime;

	clock_t last_loc;

	// tmp
	bool first_frame2;
	Mat first_frame;
};

